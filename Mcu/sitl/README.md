# AM32 bootloader SITL

Runs the unmodified bootloader as a native Linux process for testing
the input-type detection, the 19200 baud 4-way configuration protocol
and the DroneCAN bootloader flows without hardware.

Build and run:

```
make AM32_SITL_BOOTLOADER_PB4_CAN
./obj/AM32_SITL_BOOTLOADER_PB4_CAN_*.elf --eeprom test_ee.bin --can-uri none
```

Run with `--help` for all options. Key points:

- the signal wire is driven over UDP with the same input protocol as
  the main firmware SITL (see `Mcu/SITL/README.md` in the am32-firmware
  repo): PWM/DShot frames, type 4 packets carrying raw 19200 serial
  bytes, and type 5 packets setting a constant line state. The
  bootloader's bit-banged replies come back as type 4 packets.
  `--line low|high|float` sets the line state at boot, as if an FC or
  config adapter is already attached.
- simulated time advances deterministically with the bootloader's timer
  and GPIO accesses, so the bit-banged serial timing is exact and test
  runs are repeatable; `--speedup` scales against the wall clock.
- 128KB of flash is backed by `<eeprom>.blflash`, mapped at the real
  `0x08000000` (the build is `-no-pie` for this). On first run it is
  seeded with a minimal valid application image so the boot checks
  behave like a programmed ESC. The eeprom page is kept coherent with
  the `--eeprom` file shared with the main firmware SITL.
- DroneCAN runs over multicast UDP (`mcast:N`, wire compatible with the
  firmware SITL, pydronecan and ArduPilot SITL). RTC backup registers
  (the firmware-update handoff) are backed by `<eeprom>.bkup`.
- `jump_to_application()` execs the vector given after `--` on the
  command line; without one it exits with code 42 ("would have
  booted"). `NVIC_SystemReset()` re-execs the bootloader with
  `--reset-cause software`. The main firmware SITL's `--bootloader`
  option chains the two, giving the full hardware-like boot loop.

Tests (the python protocol tools live in the am32-firmware repo):

```
python3 Mcu/sitl/test/run_bl_tests.py \
    --bootloader obj/AM32_SITL_BOOTLOADER_PB4_CAN_*.elf \
    --fw-tools ../AM32/Mcu/SITL \
    [--app-elf ../AM32/obj/AM32_AM32_SITL_CAN_*.elf]
```

`--app-elf` enables the full execve boot-chain tests (bootloader to
firmware and back through resets). CI runs the bootloader-only subset.
