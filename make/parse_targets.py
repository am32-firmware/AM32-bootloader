#!/usr/bin/env python3
'''
parse Inc/targets.h and emit the custom board build targets for the Makefile.

Each board is a "#ifdef <BOARDNAME> ... #endif" block containing at least:
    #define FILE_NAME  "<name>"   the MCU is the name token matching a known MCU
                                  family, a trailing _CAN marks a CAN build
    #define TARGET_TAG <tag>      short tag used in the generated target name

For the ARK_G431_CAN block this produces the build target
AM32_G431_BOOTLOADER_ARKG4_CAN.

Usage:
    parse_targets.py builds   -> one line per board: BUILD|TAG|BOARDDEFINE
                                 e.g. G431_CAN|ARKG4|ARK_G431_CAN
    parse_targets.py mcus     -> unique MCU families used by custom boards
'''

import os
import re
import sys

# AM32 MCU families the bootloader knows about (matches Makefile MCU_BUILDS)
KNOWN_MCUS = ['E230', 'F031', 'F051', 'F415', 'F421', 'G071', 'G431',
              'L431', 'V203', 'A153']

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGETS_H = os.path.join(REPO_ROOT, 'Inc', 'targets.h')


def is_commented(line, marker):
    '''true if marker appears only after a // comment on this line'''
    idx = line.find(marker)
    pre = line[:idx]
    return '//' in pre


def parse_boards(path):
    '''return list of dicts {guard, file_name, tag} for each board block'''
    boards = []
    cur = None
    depth = 0
    with open(path) as f:
        for line in f:
            s = line.strip()
            if s.startswith('#ifdef') or s.startswith('#ifndef') or s.startswith('#if'):
                depth += 1
                if depth == 1 and s.startswith('#ifdef'):
                    parts = s.split()
                    cur = {'guard': parts[1], 'file_name': None, 'tag': None}
                continue
            if s.startswith('#endif'):
                if depth == 1 and cur is not None:
                    if cur['file_name']:
                        boards.append(cur)
                    cur = None
                depth = max(0, depth - 1)
                continue
            if cur is None:
                continue
            if '#define' in line and 'FILE_NAME' in line:
                if is_commented(line, '#define') or 'DISABLE_BUILD' in line:
                    continue
                m = re.search(r'"([^"]+)"', line)
                if m:
                    cur['file_name'] = m.group(1)
            elif '#define' in line and 'TARGET_TAG' in line:
                if is_commented(line, '#define'):
                    continue
                m = re.search(r'#define\s+TARGET_TAG\s+(\S+)', line)
                if m:
                    cur['tag'] = m.group(1)
    return boards


def board_build(board):
    '''return (build, tag, boarddefine) or None if the MCU is unrecognised'''
    tokens = board['file_name'].split('_')
    mcu = next((t for t in tokens if t in KNOWN_MCUS), None)
    if mcu is None:
        sys.stderr.write(
            "parse_targets.py: no known MCU in FILE_NAME '%s', skipping\n"
            % board['file_name'])
        return None
    is_can = 'CAN' in tokens
    build = mcu + ('_CAN' if is_can else '')
    tag = board['tag'] if board['tag'] else board['file_name']
    return (build, tag, board['guard'])


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'builds'
    if not os.path.exists(TARGETS_H):
        return
    builds = [b for b in (board_build(x) for x in parse_boards(TARGETS_H)) if b]
    if mode == 'mcus':
        mcus = []
        for build, _, _ in builds:
            mcu = build.split('_')[0]
            if mcu not in mcus:
                mcus.append(mcu)
        print(' '.join(mcus))
    else:  # builds
        for build, tag, define in builds:
            print('%s|%s|%s' % (build, tag, define))


if __name__ == '__main__':
    main()
