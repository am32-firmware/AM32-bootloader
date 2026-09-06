#!/usr/bin/env python3
'''
check the layout of a transition bootloader updater hex

The transition updater is flashed by a legacy 4k bootloader over the top of
the application, so getting its layout wrong would leave an ESC that needs a
debugger to recover. Check the invariants it relies on.
'''

import struct
import sys
from argparse import ArgumentParser

LEGACY_APP_START = 0x08001000  # where a 4k bootloader starts the application
CODE_START = 0x08004000        # first address above the new 16k bootloader
LEGACY_EEPROM = 0x0800F800     # eeprom of the 64k flash layout
RAM_START = 0x20000000
RAM_END = RAM_START + 112*1024

APP_SIGNATURE_MAGIC1 = 0x68f058e6
APP_SIGNATURE_MAGIC2 = 0xafcee5a0


def parse_hex(path):
    '''parse an intel hex file, filling gaps with 0xff as the flashing clients do'''
    upper = 0
    origin = None
    next_address = 0
    img = bytearray()
    for line in open(path, "r"):
        line = line.strip()
        if line == "":
            continue
        rec = bytes.fromhex(line[1:])
        count, rtype, data = rec[0], rec[3], rec[4:4+rec[0]]
        address = upper + ((rec[1] << 8) | rec[2])
        if rtype == 0x00:
            if origin is None:
                origin = address
                next_address = address
            if address < next_address:
                raise Exception("out of order record at 0x%08x" % address)
            img += b'\xff' * (address - next_address)
            img += data
            next_address = address + count
        elif rtype == 0x04:
            upper = ((data[0] << 8) | data[1]) << 16
    if origin is None:
        raise Exception("no data records")
    return origin, bytes(img)


def check(path):
    origin, img = parse_hex(path)
    end = origin + len(img)
    if origin != LEGACY_APP_START:
        raise Exception("origin is 0x%08x, expected 0x%08x" % (origin, LEGACY_APP_START))
    if end > LEGACY_EEPROM:
        raise Exception("image ends at 0x%08x, past the legacy eeprom 0x%08x" % (end, LEGACY_EEPROM))

    stack, entry = struct.unpack("<II", img[:8])
    if stack < RAM_START or stack > RAM_END:
        raise Exception("stack pointer 0x%08x is not in ram" % stack)
    if (entry & 1) == 0:
        raise Exception("entry point 0x%08x is not thumb" % entry)
    if (entry & ~1) < CODE_START or (entry & ~1) >= end:
        raise Exception("entry point 0x%08x is not in the code region" % entry)

    # everything below the code must be the flashing client's gap fill, else
    # the updater would be erasing its own code as it runs
    if set(img[8:CODE_START-origin]) not in ({0xff}, set()):
        raise Exception("image has content below 0x%08x" % CODE_START)

    # the updater is left behind in the application area, so the new
    # bootloader must never accept it as an application
    sig = struct.pack("<II", APP_SIGNATURE_MAGIC1, APP_SIGNATURE_MAGIC2)
    ofs = img.find(sig, CODE_START-origin)
    if ofs < 0:
        raise Exception("no app signature found")
    fwlen, = struct.unpack("<I", img[ofs+8:ofs+12])
    if fwlen != 0xFFFFFFFF:
        raise Exception("app signature length is %#x, expected an unbootable 0xffffffff" % fwlen)


parser = ArgumentParser(description=__doc__)
parser.add_argument("hex", help="transition updater hex file")
args = parser.parse_args()

try:
    check(args.hex)
except Exception as ex:
    print("%s: %s" % (args.hex, ex))
    sys.exit(1)
