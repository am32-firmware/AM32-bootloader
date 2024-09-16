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
}

bname = os.path.basename(args.hex)
a = bname.split("_")
if a[0] != 'AM32' or a[2] != 'BL' or a[3] != "UPDATER" or not a[-1].endswith(".hex"):
    print("Bad hex file name")
    sys.exit(1)
MCU = a[1]
PIN = a[4]
VER = a[-1][:-4]

if len(a) == 6:
    flash_size = "%uK" % default_flash_sizes[MCU]
elif len(a) == 7:
    flash_size = a[-2]
else:
    print("Bad hex file name2")
    sys.exit(1)
if not MCU in 'E230 F031 F051 F415 F415_128K F421 G071 G071_64K L431 L431_128K G431 V203'.split():
    print(f"Bad MCU {MCU}")
    sys.exit(1)

d = {
    "type": args.type,
    "mcuType": MCU,
    "pin": PIN,
    "githash": args.githash,
    "version": VER,
    "flashSize": flash_size,
    "hex": base64.b64encode(img).decode('utf-8'),
}

f = open(args.amj, "w")
f.write(json.dumps(d, indent=4))
f.close()
