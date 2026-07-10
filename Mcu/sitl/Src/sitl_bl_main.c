/*
  sitl_bl_main.c - process entry for the AM32 bootloader SITL

  Parses the command line, sets up the flash mapping, pin model and
  instance lock, then runs the unmodified bootloader main
  (am32_bl_main, renamed via Mcu/sitl/Inc/main.h).

  Everything after a "--" on the command line is the exec vector of the
  application firmware: jump_to_application() execs it (with the
  AM32_SITL_FROM_BL environment marker so the app does not immediately
  exec back into the bootloader). Without an app vector a jump exits
  with code 42, which the test harness treats as "would have booted".
 */

#define _GNU_SOURCE
#include "sitl_bl.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

sitl_bl_config_t sitl_bl_cfg = {
    .eeprom_path = "am32_eeprom.bin",
    .flash_path = NULL,
    .input_port = 57733,
    .can_uri = "mcast:0",
    .speedup = 1.0f,
    .bind_any = false,
    .uid = NULL,
    .initial_line = -1,
    .reset_cause = SITL_RESET_POWER,
    .verbose = false,
    .app_argv = NULL,
};

static char** saved_argv;
static int saved_argc;

extern int am32_bl_main(void);

void Error_Handler(void)
{
    fprintf(stderr, "SITL: Error_Handler\n");
    exit(1);
}

int sitl_bl_udp_socket(void)
{
    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    return fd;
}

bool sitl_bl_was_software_reset(void)
{
    return sitl_bl_cfg.reset_cause == SITL_RESET_SOFTWARE;
}

void sitl_bl_jump_app(void)
{
    fflush(NULL);
    if (sitl_bl_cfg.app_argv == NULL || sitl_bl_cfg.app_argv[0] == NULL) {
        fprintf(stderr, "SITL: would jump to application (no app vector)\n");
        exit(42);
    }
    fprintf(stderr, "SITL: jumping to application %s\n", sitl_bl_cfg.app_argv[0]);
    setenv("AM32_SITL_FROM_BL", "1", 1);
    execv(sitl_bl_cfg.app_argv[0], sitl_bl_cfg.app_argv);
    perror("SITL: exec application");
    _exit(1);
}

void sitl_bl_system_reset(void)
{
    // a reset always lands back in the bootloader, with the software
    // reset cause visible as on hardware
    fflush(NULL);
    char** argv = calloc(saved_argc + 3, sizeof(char*));
    int n = 0;
    bool have_cause = false;
    for (int i = 0; i < saved_argc; i++) {
        argv[n++] = saved_argv[i];
        if (strcmp(saved_argv[i], "--reset-cause") == 0 && i + 1 < saved_argc) {
            argv[n++] = "software";
            have_cause = true;
            i++; // skip old value
        }
    }
    if (!have_cause) {
        // insert before any -- separator
        int sep = n;
        for (int i = 0; i < n; i++) {
            if (strcmp(argv[i], "--") == 0) {
                sep = i;
                break;
            }
        }
        memmove(&argv[sep + 2], &argv[sep], (n - sep) * sizeof(char*));
        argv[sep] = "--reset-cause";
        argv[sep + 1] = "software";
        n += 2;
    }
    argv[n] = NULL;
    execv(argv[0], argv);
    perror("SITL: exec self");
    _exit(1);
}

// RTC backup registers: file backed so they survive the execve chain,
// like the battery backed domain on hardware
static char bkup_path[512];

uint32_t sitl_bl_bkup_read(uint8_t idx)
{
    uint32_t v = 0;
    FILE* f = fopen(bkup_path, "rb");
    if (f != NULL) {
        fseek(f, idx * 4, SEEK_SET);
        if (fread(&v, 4, 1, f) != 1) {
            v = 0;
        }
        fclose(f);
    }
    return v;
}

void sitl_bl_bkup_write(uint8_t idx, uint32_t value)
{
    FILE* f = fopen(bkup_path, "r+b");
    if (f == NULL) {
        f = fopen(bkup_path, "w+b");
    }
    if (f == NULL) {
        return;
    }
    fseek(f, idx * 4, SEEK_SET);
    fwrite(&value, 4, 1, f);
    fclose(f);
}

static void lock_instance(void)
{
    // same lock file as the main firmware SITL: the fw and bootloader
    // of one chain never run concurrently, a second instance fails
    static char lock_path[512];
    snprintf(lock_path, sizeof(lock_path), "%s.lock", sitl_bl_cfg.eeprom_path);
    const int fd = open(lock_path, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (fd < 0) {
        perror(lock_path);
        exit(1);
    }
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        fprintf(stderr, "SITL: eeprom %s in use by another SITL instance\n",
            sitl_bl_cfg.eeprom_path);
        exit(1);
    }
    // fd deliberately kept open, dropped on exec
}

static void usage(const char* prog)
{
    printf("Usage: %s [options] [-- app args...]\n"
           "  --eeprom FILE      shared eeprom file (default am32_eeprom.bin)\n"
           "  --flash FILE       flash backing file (default <eeprom>.blflash)\n"
           "  --input-port N     UDP input port (default 57733)\n"
           "  --can-uri URI      mcast:N[:iface] or none (default mcast:0)\n"
           "  --speedup X        simulation speed, 0 = free run (default 1.0)\n"
           "  --bind-any         bind on all interfaces, not loopback only\n"
           "  --uid STR          unique ID seed, must match the firmware\n"
           "  --line S           line state at boot: low, high or float\n"
           "                     (default float), as if an FC or config\n"
           "                     adapter is already attached\n"
           "  --reset-cause C    power|software|watchdog (default power)\n"
           "  --verbose\n"
           "  everything after -- is the exec vector for the application\n",
        prog);
}

int main(int argc, char** argv)
{
    saved_argv = argv;
    static const struct option opts[] = {
        { "eeprom", required_argument, NULL, 'e' },
        { "flash", required_argument, NULL, 'f' },
        { "input-port", required_argument, NULL, 'p' },
        { "can-uri", required_argument, NULL, 'u' },
        { "speedup", required_argument, NULL, 's' },
        { "bind-any", no_argument, NULL, 'A' },
        { "uid", required_argument, NULL, 'U' },
        { "line", required_argument, NULL, 'L' },
        { "reset-cause", required_argument, NULL, 'r' },
        { "verbose", no_argument, NULL, 'v' },
        { "help", no_argument, NULL, 'h' },
        { NULL, 0, NULL, 0 },
    };
    int c;
    while ((c = getopt_long(argc, argv, "e:f:p:u:s:AU:L:r:vh", opts, NULL)) != -1) {
        switch (c) {
        case 'e':
            sitl_bl_cfg.eeprom_path = optarg;
            break;
        case 'f':
            sitl_bl_cfg.flash_path = optarg;
            break;
        case 'p':
            sitl_bl_cfg.input_port = atoi(optarg);
            break;
        case 'u':
            sitl_bl_cfg.can_uri = optarg;
            break;
        case 's':
            sitl_bl_cfg.speedup = strtof(optarg, NULL);
            break;
        case 'A':
            sitl_bl_cfg.bind_any = true;
            break;
        case 'U':
            sitl_bl_cfg.uid = optarg;
            break;
        case 'L':
            if (strcmp(optarg, "low") == 0) {
                sitl_bl_cfg.initial_line = 0;
            } else if (strcmp(optarg, "high") == 0) {
                sitl_bl_cfg.initial_line = 1;
            } else {
                sitl_bl_cfg.initial_line = -1;
            }
            break;
        case 'r':
            if (strcmp(optarg, "software") == 0) {
                sitl_bl_cfg.reset_cause = SITL_RESET_SOFTWARE;
            } else if (strcmp(optarg, "watchdog") == 0) {
                sitl_bl_cfg.reset_cause = SITL_RESET_WATCHDOG;
            } else {
                sitl_bl_cfg.reset_cause = SITL_RESET_POWER;
            }
            break;
        case 'v':
            sitl_bl_cfg.verbose = true;
            break;
        case 'h':
        default:
            usage(argv[0]);
            exit(c == 'h' ? 0 : 1);
        }
    }
    // find the -- separator ourselves: getopt_long stops there and
    // optind points at the first following arg
    if (optind < argc) {
        sitl_bl_cfg.app_argv = &argv[optind];
    }
    saved_argc = argc;

    static char flash_path[512];
    if (sitl_bl_cfg.flash_path == NULL) {
        snprintf(flash_path, sizeof(flash_path), "%s.blflash", sitl_bl_cfg.eeprom_path);
        sitl_bl_cfg.flash_path = flash_path;
    }
    snprintf(bkup_path, sizeof(bkup_path), "%s.bkup", sitl_bl_cfg.eeprom_path);

    fprintf(stderr, "AM32 bootloader SITL: eeprom=%s flash=%s can=%s cause=%d\n",
        sitl_bl_cfg.eeprom_path, sitl_bl_cfg.flash_path, sitl_bl_cfg.can_uri,
        sitl_bl_cfg.reset_cause);

    lock_instance();
    sitl_bl_flash_init();
    sitl_bl_time_init();
    sitl_bl_pin_init();

    return am32_bl_main();
}
