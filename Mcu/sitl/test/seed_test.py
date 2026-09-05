#!/usr/bin/env python3
"""Check real bootloader startup and recovery of old synthetic flash images."""
import argparse
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import zlib


APP = 0x4000
SIGNATURE = APP + 7168
CRC1 = SIGNATURE + 12
EEPROM = 0x1f800


def legacy_image(settings):
    # Reproduce the image shipped before the firmware name was included
    # in CRC1. Keep the historical CRC literal so this fixture does not
    # accidentally follow changes to the seeding implementation.
    image = bytearray(b'\xff' * (128 * 1024))
    image[APP:APP + 8192] = bytes(8192)
    struct.pack_into('<II', image, APP, 0x20004000, 0x08004101)
    image[APP + 512:APP + 525] = b'AM32_SITL_CAN\0'
    struct.pack_into('<IIIII16sII', image, SIGNATURE,
                     0x68f058e6, 0xafcee5a0, 8192, 0x62ac7feb, 0,
                     b'SITL', 0, 0)
    image[EEPROM:EEPROM + len(settings)] = settings
    return image


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bootloader', required=True)
    ap.add_argument('--can-uri', default='none')
    args = ap.parse_args()
    binary = str(Path(args.bootloader).resolve())
    with tempfile.TemporaryDirectory(prefix='am32-bl-seed-') as tmp:
        ee = Path(tmp) / 'settings.bin'
        # Recognisable settings bytes; recovery must preserve every byte.
        settings = bytes([1]) + bytes(range(1, 192))
        ee.write_bytes(settings)
        flash = Path(str(ee) + '.blflash')

        def boot(expect_jump=True):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
                reservation.bind(('127.0.0.1', 0))
                port = reservation.getsockname()[1]
            proc = subprocess.Popen([binary, '--eeprom', str(ee),
                                     '--input-port', str(port), '--can-uri', args.can_uri],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            try:
                try:
                    output = proc.communicate(timeout=10 if expect_jump else 3)[0]
                except subprocess.TimeoutExpired:
                    assert not expect_jump, 'bootloader did not jump to the application'
                    proc.terminate()
                    output = proc.communicate(timeout=5)[0]
                else:
                    assert expect_jump and proc.returncode == 42, output.decode(errors='replace')
                assert ee.read_bytes() == settings, 'EEPROM settings changed'
                return output
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.communicate()

        boot()
        fresh = flash.read_bytes()
        crc = zlib.crc32(fresh[APP:SIGNATURE], 0xffffffff) ^ 0xffffffff
        assert struct.unpack_from('<I', fresh, CRC1)[0] == crc
        print('PASS: freshly seeded application passes the real boot gate')

        legacy = legacy_image(settings)
        legacy[0x100] = 0x42  # unrelated flash must survive recovery
        flash.write_bytes(legacy)
        assert b'repaired legacy' in boot()
        expected = bytearray(legacy)
        struct.pack_into('<I', expected, CRC1, crc)
        assert flash.read_bytes() == expected, 'recovery changed more than CRC1'
        assert b'repaired legacy' not in boot()
        assert flash.read_bytes() == expected
        print('PASS: legacy CRC repaired once, preserving settings and unrelated flash')

        # An uploaded or damaged image must still fail validation, even
        # if it resembles the old dummy image and has the same stale CRC.
        legacy[APP + 1024] ^= 1
        flash.write_bytes(legacy)
        assert b'repaired legacy' not in boot(expect_jump=False)
        assert flash.read_bytes() == legacy
        print('PASS: modified firmware is neither repaired nor booted')


if __name__ == '__main__':
    main()
