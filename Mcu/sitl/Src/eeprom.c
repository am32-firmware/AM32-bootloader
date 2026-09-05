/*
  eeprom.c for the bootloader SITL: 128KB of flash backed by a file and
  mapped at the real MCU flash address 0x08000000, so the unmodified
  direct pointer reads in bootloader/main.c and DroneCAN.c (app vector
  checks, signature scan, eeprom magic) work as on hardware.

  The eeprom page inside the flash image is kept coherent with the main
  firmware SITL's separate eeprom file: mirrored from the file at
  startup (the file is authoritative, the firmware may have changed
  settings since the bootloader last ran) and written through on every
  eeprom range flash write.
 */

#define _GNU_SOURCE // MAP_FIXED_NOREPLACE
#include "sitl_bl.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#define FLASH_BASE_ADDR 0x08000000UL
#define FLASH_SIZE (128 * 1024)
#define PAGE_SIZE_BYTES 2048
// must match EEPROM_START_ADD for BOARD_FLASH_SIZE=128 in main.c
#define EEPROM_OFFSET 0x1F800
#define APP_OFFSET 0x4000 // FIRMWARE_RELATIVE_START for CAN builds
#define SEEDED_APP_SIZE (8 * 1024)

static uint8_t* flash;
static int flash_fd = -1;

// same crc32 as bootloader/DroneCAN/DroneCAN.c, for the seeded app
// signature
static uint32_t bl_crc32(const uint8_t* buf, uint32_t size)
{
    uint32_t crc = 0;
    while (size--) {
        crc ^= *buf++;
        for (uint8_t i = 0; i < 8; i++) {
            const uint32_t mask = -(crc & 1);
            crc >>= 1;
            crc ^= (0xEDB88320 & mask);
        }
    }
    return crc;
}

/*
  seed a minimal valid application image: vector table that passes the
  jump() stack/entry checks and an app_signature block that passes
  DroneCAN_boot_ok(), so the boot gates behave as with a programmed ESC
 */
static void seed_app(uint8_t* app, bool legacy)
{
    const uint32_t app_len = SEEDED_APP_SIZE;
    memset(app, 0, app_len);
    const uint32_t sp = 0x20004000; // inside RAM
    const uint32_t entry = FLASH_BASE_ADDR + APP_OFFSET + 0x101;
    memcpy(app, &sp, 4);
    memcpy(app + 4, &entry, 4);

    // Include the name in CRC1. Writing it after the signature was
    // calculated left the simulated application unable to pass boot checks.
    static const char fwname[] = "AM32_SITL_CAN";
    if (!legacy) {
        memcpy(app + 512, fwname, sizeof(fwname));
    }

    // app_signature at an aligned offset near the end of the image,
    // layout from bootloader/DroneCAN/DroneCAN.c
    struct __attribute__((packed)) {
        uint32_t magic1, magic2, fwlen, crc1, crc2;
        char mcu[16];
        uint32_t unused[2];
    } sig;
    memset(&sig, 0, sizeof(sig));
    sig.magic1 = 0x68f058e6;
    sig.magic2 = 0xafcee5a0;
    sig.fwlen = app_len;
    strncpy(sig.mcu, "SITL", sizeof(sig.mcu) - 1);
    const uint32_t sig_ofs = app_len - 1024; // 8 byte aligned
    sig.crc1 = bl_crc32(app, sig_ofs);
    memcpy(app + sig_ofs, &sig, sizeof(sig));
    // crc2 covers from end of signature to end of fw
    sig.crc2 = bl_crc32(app + sig_ofs + sizeof(sig), app_len - (sig_ofs + sizeof(sig)));
    memcpy(app + sig_ofs, &sig, sizeof(sig));

    // Reproduce the old synthetic image only to recognise it for repair.
    if (legacy) {
        memcpy(app + 512, fwname, sizeof(fwname));
    }
}

static void seed_flash(void)
{
    memset(flash, 0xFF, FLASH_SIZE);
    seed_app(flash + APP_OFFSET, false);
}

static void repair_legacy_seed(void)
{
    uint8_t legacy[SEEDED_APP_SIZE];
    seed_app(legacy, true);
    // Match the entire old synthetic application, not merely a bad CRC:
    // never repair or replace firmware uploaded through the configurator.
    if (memcmp(flash + APP_OFFSET, legacy, sizeof(legacy)) == 0) {
        seed_app(flash + APP_OFFSET, false);
        fprintf(stderr, "SITL: repaired legacy seeded application CRC\n");
    }
}

void sitl_bl_flash_init(void)
{
    const char* path = sitl_bl_cfg.flash_path;
    bool fresh = false;
    flash_fd = open(path, O_RDWR | O_CLOEXEC);
    if (flash_fd < 0) {
        flash_fd = open(path, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
        fresh = true;
    }
    if (flash_fd < 0) {
        perror(path);
        exit(1);
    }
    struct stat st;
    if (fstat(flash_fd, &st) == 0 && st.st_size < FLASH_SIZE) {
        fresh = fresh || st.st_size == 0;
        if (ftruncate(flash_fd, FLASH_SIZE) != 0) {
            perror("ftruncate flash");
            exit(1);
        }
    }
    // MAP_FIXED_NOREPLACE fails rather than clobbering an existing
    // mapping at 0x08000000. Where it is unavailable, fall back to a
    // plain hinted mmap (NOT MAP_FIXED, which would silently replace
    // whatever is there) and verify the kernel honoured the hint
#ifdef MAP_FIXED_NOREPLACE
    const int fixed_flag = MAP_FIXED_NOREPLACE;
#else
    const int fixed_flag = 0;
#endif
    flash = mmap((void*)FLASH_BASE_ADDR, FLASH_SIZE, PROT_READ | PROT_WRITE,
        MAP_SHARED | fixed_flag, flash_fd, 0);
    if (flash != (void*)FLASH_BASE_ADDR) {
        perror("SITL: cannot map flash at 0x08000000");
        fprintf(stderr, "SITL: build must be -no-pie for the fixed flash mapping\n");
        exit(1);
    }
    if (fresh) {
        fprintf(stderr, "SITL: seeding flash image %s\n", path);
        seed_flash();
    } else {
        repair_legacy_seed();
    }

    // mirror the shared eeprom file into the eeprom page: the app may
    // have changed settings since the bootloader last ran
    FILE* f = fopen(sitl_bl_cfg.eeprom_path, "rb");
    if (f != NULL) {
        uint8_t buf[PAGE_SIZE_BYTES];
        const size_t n = fread(buf, 1, sizeof(buf), f);
        fclose(f);
        if (n > 0) {
            memcpy(flash + EEPROM_OFFSET, buf, n);
        }
    }
    msync(flash, FLASH_SIZE, MS_ASYNC);
}

/*
  write through eeprom range changes to the shared eeprom file
 */
static void eeprom_write_through(uint32_t offset, uint32_t length)
{
    if (offset + length <= EEPROM_OFFSET || offset >= EEPROM_OFFSET + PAGE_SIZE_BYTES) {
        return;
    }
    const uint32_t start = offset > EEPROM_OFFSET ? offset : EEPROM_OFFSET;
    uint32_t end = offset + length;
    if (end > EEPROM_OFFSET + PAGE_SIZE_BYTES) {
        end = EEPROM_OFFSET + PAGE_SIZE_BYTES;
    }
    FILE* f = fopen(sitl_bl_cfg.eeprom_path, "r+b");
    if (f == NULL) {
        f = fopen(sitl_bl_cfg.eeprom_path, "w+b");
    }
    if (f == NULL) {
        perror(sitl_bl_cfg.eeprom_path);
        return;
    }
    fseek(f, start - EEPROM_OFFSET, SEEK_SET);
    fwrite(flash + start, 1, end - start, f);
    fclose(f);
}

bool save_flash_nolib(const uint8_t* data, uint32_t length, uint32_t add)
{
    if (add < FLASH_BASE_ADDR || add + length > FLASH_BASE_ADDR + FLASH_SIZE) {
        return false;
    }
    const uint32_t offset = add - FLASH_BASE_ADDR;
    // page erase when writing to a page boundary, as the real driver
    if ((offset % PAGE_SIZE_BYTES) == 0) {
        uint32_t erase = FLASH_SIZE - offset;
        if (erase > PAGE_SIZE_BYTES) {
            // erase all pages the write spans, like the l431 driver
            erase = ((length + PAGE_SIZE_BYTES - 1) / PAGE_SIZE_BYTES) * PAGE_SIZE_BYTES;
        }
        memset(flash + offset, 0xFF, erase);
    }
    memcpy(flash + offset, data, length);
    msync(flash + (offset & ~4095UL), ((length + 4095) & ~4095UL) + 4096, MS_ASYNC);
    eeprom_write_through(offset, length);
    return memcmp(flash + offset, data, length) == 0;
}

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    memcpy(data, (const void*)(uintptr_t)add, out_buff_len);
}
