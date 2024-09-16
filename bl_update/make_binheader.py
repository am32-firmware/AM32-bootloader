#!/usr/bin/env python3
'''
convert a binary file to a C header
'''

import sys

def binary_to_c_header(binary_filename, header_filename, array_name="bl_image", pad=8):
    """
    Reads a binary file and writes a C header file with a constant uint8_t array.

    :param binary_filename: The path to the binary file to read.
    :param header_filename: The path to the C header file to write.
    :param array_name: The name of the array variable in the generated C code.
    """
    try:
        # Read the binary file
        with open(binary_filename, 'rb') as bin_file:
            binary_data = bin_file.read()

        # possibly pad with zeros
        if len(binary_data) % pad != 0:
            pad_len = pad - len(binary_data) % pad
            pad_bytes = bytes([0]*pad_len)
            binary_data += pad_bytes

        # Open the header file for writing
        with open(header_filename, 'w') as header_file:
            # Write header guard
            guard_macro = '{}_H'.format(array_name.upper())
            header_file.write(f'// generated from {binary_filename}\n')
            header_file.write('#pragma once\n')
            header_file.write('#include <stdint.h>\n\n')

            # Write the array declaration
            header_file.write('static const uint8_t {}[] = {{\n'.format(array_name))

            # Convert binary data to a list of hex strings
            hex_values = ['0x{:02X}'.format(b) for b in binary_data]

            # Define how many values to include per line
            line_length = 16  # Adjust this for more or fewer per line

            # Write the hex values into the array
            for i in range(0, len(hex_values), line_length):
                line = ', '.join(hex_values[i:i+line_length])
                if i + line_length >= len(hex_values):
                    # Last line without trailing comma
                    header_file.write('    {}\n'.format(line))
                else:
                    header_file.write('    {},\n'.format(line))

            header_file.write('};\n\n')

        print("Wrote {}".format(header_filename))

    except IOError as e:
        print("An I/O error occurred: {}".format(e))
        sys.exit(1)

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("binfile", help="binary input file")
parser.add_argument("header", help="C header output file")
args = parser.parse_args()

binary_to_c_header(args.binfile, args.header)
sys.exit(0)
