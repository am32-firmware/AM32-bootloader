/*
  sitl_bl_pin.c - signal wire model for the bootloader SITL

  The host drives the wire with UDP packets on the input port (same
  wire format as the main firmware SITL, see Mcu/SITL/README.md in the
  am32-firmware repo):
    types 0-3: PWM/DShot frames, synthesized into timed edges
    type 4:    raw 19200 8N1 serial bytes
    type 5:    constant line state (driven high/low, or floating)

  The bootloader reads the wire with gpio_read() at simulated times and
  bit-bangs replies by driving the pin; its output edges are decoded
  back into bytes and sent as type 4 packets to the most recent sender.
 */

#include "sitl_bl.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define SITL_INPUT_MAGIC 0x4453
#define TYPE_PWM 0
#define TYPE_DSHOT150 1
#define TYPE_DSHOT300 2
#define TYPE_DSHOT600 3
#define TYPE_SERIAL 4
#define TYPE_LINE 5

#define FLAG_IDLE_HIGH 0x0001
#define FLAG_FLOATING 0x0002
#define FLAG_GAP 0x0004 // leading 1ms line idle before the bytes

#define SERIAL_BIT_NS 52083 // 19200 baud
#define READ_GRANT_GPIO_NS 100 // sim time per gpio access
#define SERIAL_MAX 200

struct __attribute__((packed)) input_hdr {
    uint16_t magic;
    uint8_t type;
    uint8_t len;
    uint16_t flags;
};

// host driven waveform: ring of edges plus the idle state after they
// have played out
#define EDGE_RING 32768
static struct {
    uint64_t t_ns;
    uint8_t level;
} edges[EDGE_RING];
static unsigned edge_head, edge_tail; // consume at head, append at tail
static uint64_t wave_end_ns; // sim time the queued waveform ends
static uint8_t host_level = 1; // level of the last consumed edge
static bool host_seen_edge;

enum idle_state {
    IDLE_DRIVEN_LOW,
    IDLE_DRIVEN_HIGH,
    IDLE_FLOATING,
};
static enum idle_state idle = IDLE_FLOATING;

// bootloader side pin state
static bool output_mode;
static uint8_t out_level = 1;
static uint32_t pull; // GPIO_PULL_* from blutil.h (0 none, 1 up, 2 down)
static uint8_t last_line_level = 1; // charge retention for a floating pin

// bootloader TX edge capture for reply decode
#define TX_EDGES 4096
static struct {
    uint64_t t_ns;
    uint8_t level;
} tx_edges[TX_EDGES];
static unsigned tx_count;

static int fd = -1;
static struct sockaddr_in last_sender;
static bool have_sender;

void sitl_bl_pin_init(void)
{
    if (sitl_bl_cfg.initial_line == 0) {
        idle = IDLE_DRIVEN_LOW;
        last_line_level = 0;
    } else if (sitl_bl_cfg.initial_line == 1) {
        idle = IDLE_DRIVEN_HIGH;
    }
    if (sitl_bl_cfg.input_port <= 0) {
        return;
    }
    fd = sitl_bl_udp_socket();
    if (fd < 0) {
        perror("SITL: input socket");
        return;
    }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)sitl_bl_cfg.input_port);
    addr.sin_addr.s_addr = htonl(sitl_bl_cfg.bind_any ? INADDR_ANY : INADDR_LOOPBACK);
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        perror("SITL: input bind");
        close(fd);
        fd = -1;
        return;
    }
    fprintf(stderr, "SITL: bootloader input on udp port %d\n", sitl_bl_cfg.input_port);
}

static void add_edge(uint64_t t_ns, uint8_t level)
{
    const unsigned next = (edge_tail + 1) % EDGE_RING;
    if (next == edge_head) {
        return; // ring full, drop
    }
    edges[edge_tail].t_ns = t_ns;
    edges[edge_tail].level = level;
    edge_tail = next;
    if (t_ns > wave_end_ns) {
        wave_end_ns = t_ns;
    }
}

static uint64_t wave_start(void)
{
    const uint64_t now = sitl_bl_time_ns();
    return wave_end_ns > now ? wave_end_ns : now;
}

static void synth_serial(const uint8_t* data, unsigned n)
{
    uint64_t t = wave_start();
    for (unsigned i = 0; i < n; i++) {
        uint8_t b = data[i];
        add_edge(t, 0); // start bit
        t += SERIAL_BIT_NS;
        for (int bit = 0; bit < 8; bit++) {
            add_edge(t, (b >> bit) & 1);
            t += SERIAL_BIT_NS;
        }
        add_edge(t, 1); // stop bit
        t += SERIAL_BIT_NS;
    }
    wave_end_ns = t;
}

static void synth_pwm(uint16_t width_us, uint8_t idle_level)
{
    uint64_t t = wave_start();
    add_edge(t, 1);
    add_edge(t + (uint64_t)width_us * 1000ULL, idle_level);
    wave_end_ns = t + (uint64_t)width_us * 1000ULL;
}

static void synth_dshot(uint8_t type, uint16_t frame, uint8_t idle_level)
{
    uint32_t bit_ns;
    switch (type) {
    case TYPE_DSHOT150:
        bit_ns = 6667;
        break;
    case TYPE_DSHOT300:
        bit_ns = 3333;
        break;
    default:
        bit_ns = 1667;
        break;
    }
    uint64_t t = wave_start();
    for (int i = 15; i >= 0; i--) {
        const uint32_t high_ns = (frame & (1U << i)) ? (bit_ns * 3) / 4 : (bit_ns * 3) / 8;
        add_edge(t, 1);
        add_edge(t + high_ns, 0);
        t += bit_ns;
    }
    add_edge(t, idle_level);
    wave_end_ns = t;
}

void sitl_bl_pin_poll(void)
{
    if (fd < 0) {
        return;
    }
    for (;;) {
        uint8_t buf[6 + SERIAL_MAX];
        struct sockaddr_in src;
        socklen_t srclen = sizeof(src);
        const ssize_t ret = recvfrom(fd, buf, sizeof(buf), MSG_DONTWAIT, (struct sockaddr*)&src, &srclen);
        if (ret < (ssize_t)sizeof(struct input_hdr)) {
            return;
        }
        struct input_hdr hdr;
        memcpy(&hdr, buf, sizeof(hdr));
        if (hdr.magic != SITL_INPUT_MAGIC || hdr.type > TYPE_LINE) {
            continue;
        }
        if (sitl_bl_cfg.verbose) {
            fprintf(stderr, "SITL: pkt type=%u len=%u flags=0x%x t=%.6f\n",
                hdr.type, hdr.len, hdr.flags, sitl_bl_time_ns() * 1e-9);
        }
        const uint8_t idle_level = (hdr.flags & FLAG_IDLE_HIGH) ? 1 : 0;
        if (hdr.type == TYPE_SERIAL) {
            if (hdr.len < 1 || hdr.len > SERIAL_MAX || ret != 6 + hdr.len) {
                continue;
            }
            last_sender = src;
            have_sender = true;
            if (hdr.flags & FLAG_GAP) {
                // frame separator: the bootloader ends a frame on a
                // >5 bit inter-byte gap
                wave_end_ns = wave_start() + 1000000ULL;
            }
            synth_serial(buf + 6, hdr.len);
            idle = (hdr.flags & FLAG_FLOATING) ? IDLE_FLOATING
                : (idle_level ? IDLE_DRIVEN_HIGH : IDLE_DRIVEN_LOW);
            continue;
        }
        if (hdr.len != 4 || ret < 8) {
            continue;
        }
        uint16_t data;
        memcpy(&data, buf + 6, 2);
        if (hdr.type == TYPE_LINE) {
            idle = (hdr.flags & FLAG_FLOATING) ? IDLE_FLOATING
                : (idle_level ? IDLE_DRIVEN_HIGH : IDLE_DRIVEN_LOW);
            continue;
        }
        last_sender = src;
        have_sender = true;
        // PWM/DShot frames: the FC actively drives the line
        idle = idle_level ? IDLE_DRIVEN_HIGH : IDLE_DRIVEN_LOW;
        if (hdr.type == TYPE_PWM) {
            synth_pwm(data, idle_level);
        } else {
            synth_dshot(hdr.type, data, idle_level);
        }
    }
}

uint8_t sitl_bl_pin_read(void)
{
    const uint64_t now = sitl_bl_time_ns();
    // consume edges up to now
    while (edge_head != edge_tail && edges[edge_head].t_ns <= now) {
        host_level = edges[edge_head].level;
        host_seen_edge = true;
        edge_head = (edge_head + 1) % EDGE_RING;
    }
    uint8_t level;
    if (output_mode) {
        // reading the pin while driving it returns the driven level
        level = out_level;
    } else if (now < wave_end_ns && host_seen_edge) {
        level = host_level;
    } else {
        switch (idle) {
        case IDLE_DRIVEN_LOW:
            level = 0;
            break;
        case IDLE_DRIVEN_HIGH:
            level = 1;
            break;
        default:
            // floating: follow the pull, or keep the charge
            level = pull == 1 ? 1 : (pull == 2 ? 0 : last_line_level);
            break;
        }
    }
    last_line_level = level;
    sitl_bl_advance(READ_GRANT_GPIO_NS);
    return level;
}

void sitl_bl_pin_write(uint8_t level)
{
    if (output_mode && out_level != level && tx_count < TX_EDGES) {
        tx_edges[tx_count].t_ns = sitl_bl_time_ns();
        tx_edges[tx_count].level = level;
        tx_count++;
    }
    out_level = level;
    last_line_level = level;
    sitl_bl_advance(READ_GRANT_GPIO_NS);
}

// level the bootloader drove at time t (from the captured tx edges,
// starting high)
static uint8_t tx_level_at(uint64_t t)
{
    uint8_t level = 1;
    for (unsigned i = 0; i < tx_count; i++) {
        if (tx_edges[i].t_ns > t) {
            break;
        }
        level = tx_edges[i].level;
    }
    return level;
}

/*
  decode the captured output edges as 19200 8N1 and send the bytes as a
  type 4 reply. Called when the bootloader releases the line
  (setReceive), which ends every transmission
 */
static void flush_tx(void)
{
    uint8_t out[300];
    unsigned nout = 0;
    // find each start bit: a falling edge while the decoder is idle
    uint64_t next_start = 0;
    for (unsigned i = 0; i < tx_count && nout < sizeof(out); i++) {
        if (tx_edges[i].level != 0 || tx_edges[i].t_ns < next_start) {
            continue;
        }
        const uint64_t t0 = tx_edges[i].t_ns;
        uint8_t b = 0;
        for (int bit = 0; bit < 8; bit++) {
            const uint64_t ts = t0 + (SERIAL_BIT_NS / 2) + (bit + 1) * SERIAL_BIT_NS;
            b |= tx_level_at(ts) << bit;
        }
        // stop bit must be high, else not a byte
        if (tx_level_at(t0 + (SERIAL_BIT_NS / 2) + 9 * SERIAL_BIT_NS)) {
            out[nout++] = b;
        }
        // resume start-bit search right after the stop bit sample: the
        // bootloader's bit time quantises slightly short of ideal, so
        // the next start edge can come before a full 10 bit frame
        next_start = t0 + (SERIAL_BIT_NS / 2) + 9 * SERIAL_BIT_NS;
    }
    if (sitl_bl_cfg.verbose) {
        fprintf(stderr, "SITL: tx flush %u bytes t=%.6f\n", nout, sitl_bl_time_ns() * 1e-9);
    }
    tx_count = 0;
    if (nout == 0 || fd < 0 || !have_sender) {
        return;
    }
    for (unsigned ofs = 0; ofs < nout; ofs += SERIAL_MAX) {
        const unsigned n = nout - ofs > SERIAL_MAX ? SERIAL_MAX : nout - ofs;
        uint8_t pkt[6 + SERIAL_MAX];
        struct input_hdr hdr = {
            .magic = SITL_INPUT_MAGIC,
            .type = TYPE_SERIAL,
            .len = (uint8_t)n,
            .flags = FLAG_IDLE_HIGH,
        };
        memcpy(pkt, &hdr, sizeof(hdr));
        memcpy(pkt + 6, out + ofs, n);
        sendto(fd, pkt, 6 + n, 0, (struct sockaddr*)&last_sender, sizeof(last_sender));
    }
}

void sitl_bl_pin_mode_input(uint32_t new_pull)
{
    if (output_mode) {
        output_mode = false;
        flush_tx();
    }
    pull = new_pull;
    sitl_bl_advance(READ_GRANT_GPIO_NS);
}

void sitl_bl_pin_mode_output(void)
{
    output_mode = true;
    tx_count = 0;
    sitl_bl_advance(READ_GRANT_GPIO_NS);
}
