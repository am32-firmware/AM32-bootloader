/*
  sys_can_SITL.c - CAN over multicast UDP for the bootloader SITL.

  Wire compatible with the DroneCAN "mcast:N" URI as implemented in
  libcanard drivers/mcast/mcast.c, the AM32 firmware SITL and ArduPilot
  SITL: group 239.65.82.N port 57732, packets carrying a 10 byte header
  (magic, crc16-CCITT, flags, 29 bit message id) followed by the frame
  data. An optional interface may be given as "mcast:N:lo" or
  "mcast:N:192.168.1.5"; "none" disables CAN.

  The CAN RX "interrupt" is polled: the sim tick calls
  sitl_bl_can_poll() which drains waiting frames unless the bootloader
  has CAN IRQs disabled (as it does around each bit-banged serial
  byte); re-enabling the IRQ drains any pending frames immediately,
  like a pending interrupt firing on hardware.
 */

#if DRONECAN_SUPPORT

#include "sys_can.h"
#include "sitl_bl.h"

#include <arpa/inet.h>
#include <errno.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#define MCAST_ADDRESS_BASE "239.65.82.0"
#define MCAST_PORT 57732
#define MCAST_MAGIC 0x2934

struct __attribute__((packed)) mcast_pkt {
    uint16_t magic;
    uint16_t crc;
    uint16_t flags;
    uint32_t message_id;
    uint8_t data[CANARD_CAN_FRAME_MAX_DATA_LEN];
};
#define MCAST_HDR_LEN 10

struct CANStats canstats;

static int fd_in = -1;
static int fd_out = -1;
// our TX socket source address: multicast loopback delivers our own
// datagrams back to fd_in, but a real CAN controller never receives its
// own frames, so RX must drop them (can_seen would self-trigger and
// block the no-CAN boot fallback)
static struct sockaddr_in tx_addr;
static bool irq_enabled = true;

static uint16_t crc16_CCITT(const uint8_t* buf, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*buf++) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void sys_can_init(void)
{
    const char* name = sitl_bl_cfg.can_uri;
    if (strcmp(name, "none") == 0) {
        fprintf(stderr, "SITL: CAN disabled\n");
        return;
    }
    int bus_num = 0;
    const char* ifname = NULL;
    if (strncmp(name, "mcast:", 6) == 0 && name[6] != 0) {
        bus_num = atoi(name + 6);
        const char* colon = strchr(name + 6, ':');
        if (colon != NULL && colon[1] != 0) {
            ifname = colon + 1;
        }
    }
    if (bus_num < 0 || bus_num > 9) {
        fprintf(stderr, "SITL: invalid mcast bus %d\n", bus_num);
        exit(1);
    }

    struct in_addr if_addr;
    bool have_if = false;
    if (ifname != NULL) {
        if (inet_pton(AF_INET, ifname, &if_addr) == 1) {
            have_if = true;
        } else {
            struct ifreq ifr;
            memset(&ifr, 0, sizeof(ifr));
            strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
            const int s = socket(AF_INET, SOCK_DGRAM, 0);
            if (s < 0 || ioctl(s, SIOCGIFADDR, &ifr) != 0) {
                fprintf(stderr, "SITL: no IPv4 address for interface %s\n", ifname);
                exit(1);
            }
            if_addr = ((struct sockaddr_in*)&ifr.ifr_addr)->sin_addr;
            close(s);
            have_if = true;
        }
    }
    char address[32];
    strncpy(address, MCAST_ADDRESS_BASE, sizeof(address) - 1);
    address[sizeof(address) - 1] = 0;
    address[strlen(address) - 1] = '0' + bus_num;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(MCAST_PORT);
    inet_pton(AF_INET, address, &addr.sin_addr);

    fd_in = sitl_bl_udp_socket();
    if (fd_in < 0) {
        perror("SITL: can socket");
        exit(1);
    }
    const int one = 1;
    setsockopt(fd_in, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in bind_addr = addr;
#ifdef __CYGWIN__
    // Windows cannot bind a multicast group address.
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
#endif
    if (bind(fd_in, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) != 0) {
        perror("SITL: can bind");
        exit(1);
    }
    struct ip_mreq mreq;
    memset(&mreq, 0, sizeof(mreq));
    mreq.imr_multiaddr = addr.sin_addr;
    mreq.imr_interface.s_addr = have_if ? if_addr.s_addr : htonl(INADDR_ANY);
    if (setsockopt(fd_in, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) != 0) {
        perror("SITL: can multicast join");
        exit(1);
    }

    fd_out = sitl_bl_udp_socket();
    if (fd_out < 0) {
        perror("SITL: can tx socket");
        exit(1);
    }
    if (have_if) {
        struct sockaddr_in src;
        memset(&src, 0, sizeof(src));
        src.sin_family = AF_INET;
        src.sin_addr = if_addr;
        if (bind(fd_out, (struct sockaddr*)&src, sizeof(src)) != 0 ||
            setsockopt(fd_out, IPPROTO_IP, IP_MULTICAST_IF, &if_addr, sizeof(if_addr)) != 0) {
            perror("SITL: can tx interface");
            exit(1);
        }
    }
    if (connect(fd_out, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        perror("SITL: can tx socket");
        exit(1);
    }

    /*
      self test: verify our transmissions are delivered back to us,
      rebinding the sender to 127.0.0.1 if not (see the firmware SITL
      driver for the routing background)
     */
#ifndef __CYGWIN__
    // Cygwin poll() does not reliably report multicast. Keep Windows on
    // its normal multicast interface instead of rebinding to loopback.
    for (int attempt = have_if ? 1 : 0; attempt < 2; attempt++) {
        uint8_t probe[4] = { 0xde, 0xad, 0xbe, 0xef }; // wrong magic, ignored
        send(fd_out, probe, sizeof(probe), 0);
        struct pollfd pfd = { .fd = fd_in, .events = POLLIN, .revents = 0 };
        bool got = false;
        while (poll(&pfd, 1, 50) == 1) {
            uint8_t buf[MCAST_HDR_LEN + 64];
            const ssize_t ret = recv(fd_in, buf, sizeof(buf), MSG_DONTWAIT);
            if (ret == (ssize_t)sizeof(probe) && memcmp(buf, probe, sizeof(probe)) == 0) {
                got = true;
                break;
            }
        }
        if (got) {
            break;
        }
        if (attempt == 0) {
            close(fd_out);
            fd_out = sitl_bl_udp_socket();
            struct sockaddr_in lo_addr;
            memset(&lo_addr, 0, sizeof(lo_addr));
            lo_addr.sin_family = AF_INET;
            inet_pton(AF_INET, "127.0.0.1", &lo_addr.sin_addr);
            if (fd_out < 0 || bind(fd_out, (struct sockaddr*)&lo_addr, sizeof(lo_addr)) != 0 || connect(fd_out, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
                perror("SITL: can tx socket (loopback)");
                exit(1);
            }
            fprintf(stderr, "SITL: CAN multicast loopback failed, using 127.0.0.1 source\n");
        } else {
            fprintf(stderr, "SITL: WARNING: CAN multicast self test failed for %s\n", address);
        }
    }

#endif

    // after the self test, which may have rebound fd_out
    socklen_t alen = sizeof(tx_addr);
    getsockname(fd_out, (struct sockaddr*)&tx_addr, &alen);

    fprintf(stderr, "SITL: bootloader CAN on %s (%s:%d)\n", name, address, MCAST_PORT);
}

int16_t sys_can_transmit(const CanardCANFrame* txf)
{
    if (fd_out < 0) {
        return -1;
    }
    struct mcast_pkt pkt;
    pkt.magic = MCAST_MAGIC;
    pkt.flags = 0;
    pkt.message_id = txf->id;
    memcpy(pkt.data, txf->data, txf->data_len);
    pkt.crc = crc16_CCITT((const uint8_t*)&pkt.flags, txf->data_len + 6);
    const ssize_t ret = send(fd_out, &pkt, txf->data_len + MCAST_HDR_LEN, 0);
    if (ret < 0) {
        return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
    }
    canstats.num_tx_interrupts++;
    return 1;
}

int16_t sys_can_receive(CanardCANFrame* rx_frame)
{
    if (fd_in < 0) {
        return -1;
    }
    struct mcast_pkt pkt;
    struct sockaddr_in src;
    socklen_t srclen = sizeof(src);
    const ssize_t ret = recvfrom(fd_in, &pkt, sizeof(pkt), MSG_DONTWAIT,
                                 (struct sockaddr*)&src, &srclen);
    if (ret < 0) {
        return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
    }
    if (src.sin_port == tx_addr.sin_port
        && src.sin_addr.s_addr == tx_addr.sin_addr.s_addr) {
        return 0; // own frame looped back
    }
    if (ret < MCAST_HDR_LEN || pkt.magic != MCAST_MAGIC) {
        canstats.rxframe_error++;
        return 0;
    }
    if (pkt.crc != crc16_CCITT((const uint8_t*)&pkt.flags, ret - 4)) {
        canstats.rxframe_error++;
        return 0;
    }
    rx_frame->id = pkt.message_id;
    rx_frame->data_len = ret - MCAST_HDR_LEN;
    memcpy(rx_frame->data, pkt.data, rx_frame->data_len);
    canstats.num_rx_interrupts++;
    return 1;
}

/*
  polled CAN RX "interrupt": called from the sim tick, and on IRQ
  re-enable so a pending interrupt fires as on hardware
 */
void sitl_bl_can_poll(void)
{
    if (fd_in < 0 || !irq_enabled) {
        return;
    }
    struct pollfd pfd = { .fd = fd_in, .events = POLLIN, .revents = 0 };
    if (poll(&pfd, 1, 0) == 1) {
        DroneCAN_receiveFrame();
    }
}

void sys_can_disable_IRQ(void)
{
    irq_enabled = false;
}

void sys_can_enable_IRQ(void)
{
    irq_enabled = true;
    sitl_bl_can_poll();
}

/*
  a stable 16 byte unique ID, byte identical to the firmware SITL
  derivation so DNA node IDs match across the boot chain
 */
void sys_can_getUniqueID(uint8_t id[16])
{
    char seed[256];
    if (sitl_bl_cfg.uid != NULL) {
        strncpy(seed, sitl_bl_cfg.uid, sizeof(seed) - 1);
        seed[sizeof(seed) - 1] = 0;
    } else {
        char host[128] = "sitl";
        gethostname(host, sizeof(host) - 1);
        snprintf(seed, sizeof(seed), "%s:%s", host, sitl_bl_cfg.eeprom_path);
    }
    // FNV-1a expanded over 4 lanes
    for (int lane = 0; lane < 4; lane++) {
        uint32_t h = 2166136261U + lane * 16777619U;
        for (const char* p = seed; *p; p++) {
            h = (h ^ (uint8_t)*p) * 16777619U;
        }
        memcpy(&id[lane * 4], &h, 4);
    }
}

uint32_t get_rtc_backup_register(uint8_t idx)
{
    return sitl_bl_bkup_read(idx);
}

void set_rtc_backup_register(uint8_t idx, uint32_t value)
{
    sitl_bl_bkup_write(idx, value);
}

void setup_portpin(uint16_t portpin, bool enable)
{
    (void)portpin;
    (void)enable;
}

#endif // DRONECAN_SUPPORT
