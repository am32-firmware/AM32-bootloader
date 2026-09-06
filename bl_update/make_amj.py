#!/usr/bin/env python3
'''
Create an AM32 amj file from a *.hex firmware firmware for bootloader update image
'''

import argparse
import json
import base64
import os
import sys

parser = argparse.ArgumentParser(description='make_amj')

parser.add_argument('hex')
parser.add_argument('amj')
parser.add_argument("--type", default="bl_update")
parser.add_argument("--githash", default="unknown")

args = parser.parse_args()

img = open(args.hex, 'rb').read()

default_flash_sizes = {
    "F031" : 32,
    "F051" : 32,
    "G071" : 128,
    "E230" : 32,
    "F415" : 32,
    "F421" : 32,
    "L431" : 64,
    "G431" : 64,
    "V203" : 64,
    "A153" : 128,
}

bname = os.path.basename(args.hex)
a = bname.split("_")
if a[0] != 'AM32' or a[2] != 'BL' or a[3] != "UPDATER" or not a[-1].endswith(".hex"):
    print("Bad hex file name")
    sys.exit(1)
MCU = a[1]
PIN = a[4]
VER = a[-1][:-4]

# anything between the pin and the version is a build tag: a flash size, CAN
# for a DroneCAN bootloader, or FROM4K for an updater that a legacy 4k
# bootloader can flash
tags = a[5:-1]
can = "CAN" in tags
transition = "FROM4K" in tags
flash_sizes = [t for t in tags if t not in ("CAN", "FROM4K")]

if len(flash_sizes) > 1 or not all(t.endswith("K") for t in flash_sizes):
    print("Bad hex file name2")
    sys.exit(1)

if flash_sizes:
    flash_size = flash_sizes[0]
elif can:
    # CAN builds are always built for the 128k flash layout
    flash_size = "128K"
else:
    flash_size = "%uK" % default_flash_sizes[MCU]

if not MCU in 'E230 F031 F051 F415 F415_128K F421 G071 G071_64K L431 L431_128K G431 V203 A153'.split():
    print(f"Bad MCU {MCU}")
    sys.exit(1)

d = {
    "type": args.type,
    "mcuType": MCU,
    "pin": PIN,
    "githash": args.githash,
    "version": VER,
    "flashSize": flash_size,
    # a transition updater is flashed by a legacy 4k bootloader, over the top
    # of the application, rather than by an existing 16k CAN bootloader
    "transition": transition,
    "hex": base64.b64encode(img).decode('utf-8'),
}

f = open(args.amj, "w")
f.write(json.dumps(d, indent=4))
f.close()
