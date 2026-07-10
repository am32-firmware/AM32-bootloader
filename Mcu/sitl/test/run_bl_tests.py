#!/usr/bin/env python3
'''
test suite for the AM32 bootloader SITL.

Needs the python protocol tools from the am32-firmware repo (Mcu/SITL:
sitl_dshot.py, sitl_fourway.py); point --fw-tools at that directory.
With --app-elf (an AM32_SITL_CAN build from the same repo) the full
boot chain via execve is tested too.

usage:
  run_bl_tests.py --bootloader ../obj/AM32_SITL_BOOTLOADER_PB4_CAN_*.elf \
                  --fw-tools ../../am32-firmware/Mcu/SITL [--app-elf ...]
'''

import argparse
import glob
import os
import struct
import subprocess
import sys
import time

failures = []


def check(name, ok, detail=''):
    print('%s: %s %s' % ('PASS' if ok else 'FAIL', name, detail))
    if not ok:
        failures.append(name)
    return ok


def crc32(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = (crc >> 1) ^ (0xEDB88320 & mask)
    return crc


def make_image(size=4096):
    '''firmware image with valid vectors and app signature for mcu SITL'''
    img = bytearray(b'\xa5' * size)
    img[0:4] = struct.pack('<I', 0x20004000)
    img[4:8] = struct.pack('<I', 0x08004101)
    sig_ofs = size - 512
    sig = bytearray(struct.pack('<IIIII', 0x68f058e6, 0xafcee5a0, size, 0, 0))
    sig += b'SITL' + bytes(12) + bytes(8)
    img[sig_ofs:sig_ofs + len(sig)] = sig
    crc1 = crc32(img[:sig_ofs])
    crc2 = crc32(img[sig_ofs + len(sig):])
    img[sig_ofs + 12:sig_ofs + 16] = struct.pack('<I', crc1)
    img[sig_ofs + 16:sig_ofs + 20] = struct.pack('<I', crc2)
    return bytes(img)


class Bootloader(object):
    '''run the bootloader SITL with a private eeprom set'''

    def __init__(self, args, blargs=(), fresh=False, log='bl_test.log'):
        self.args = args
        if fresh:
            for f in ('bl_ee.bin', 'bl_ee.bin.blflash', 'bl_ee.bin.bkup',
                      'bl_ee.bin.lock'):
                if os.path.exists(f):
                    os.remove(f)
        self.log = open(log, 'ab')
        self.proc = subprocess.Popen(
            [args.bootloader, '--eeprom', 'bl_ee.bin',
             '--input-port', str(args.port)] + list(blargs),
            stdout=self.log, stderr=subprocess.STDOUT)

    def poll_exit(self, timeout, port=None, send=None):
        '''wait for exit; optionally keep sending line/dshot input'''
        deadline = time.time() + timeout
        while time.time() < deadline:
            if send is not None:
                send()
            rc = self.proc.poll()
            if rc is not None:
                return rc
            time.sleep(0.02)
        return None

    def stop(self):
        if self.proc.poll() is None:
            self.proc.kill()
        self.proc.wait()
        self.log.close()


def fourway_session(args, blargs=('--can-uri', 'none', '--line', 'high')):
    import sitl_fourway as fw
    bl = Bootloader(args, blargs)
    f = fw.FourWay(udp_port=args.port)
    time.sleep(0.3)
    return bl, f


def test_fourway(args):
    import sitl_fourway as fw
    print('--- 4way protocol')
    bl, f = fourway_session(args)
    info = f.connect(timeout=4.0)
    check('4way device info', info is not None and info[0:3] == b'471'
          and info[7] == 3, info.hex() if info else 'no reply')
    dv = f.read_devinfo_struct()
    check('4way devinfo struct', dv is not None and dv['address_shift'] == 2
          and dv['firmware_start'] << dv['address_shift'] == 0x4000,
          str(dv))

    # eeprom write via magic address, with write-through to the file
    ee = f.read_flash(200, addr16=fw.ADDR_MAGIC_EEPROM)
    check('4way eeprom read', ee is not None and len(ee) == 200)
    new_ee = bytearray(ee)
    new_ee[0] = 0x01
    new_ee[5:13] = b'SITLTEST'
    new_ee[46] = 1
    check('4way eeprom write', f.write(fw.ADDR_MAGIC_EEPROM, bytes(new_ee)))
    rb = f.read_flash(200, addr16=fw.ADDR_MAGIC_EEPROM)
    expect = bytearray(new_ee)
    expect[2] = rb[2]  # bootloader stamps its version byte
    check('4way eeprom readback', rb == bytes(expect))
    with open('bl_ee.bin', 'rb') as fh:
        filedata = fh.read()
    check('4way eeprom write-through', filedata[5:13] == b'SITLTEST',
          filedata[:16].hex())

    # app flash write + verify
    blob = bytes(range(256))
    check('4way flash write', f.write(0x1000, blob))
    rd = f.read_flash(256, addr16=0x1000)
    check('4way flash readback', rd == blob)
    with open('bl_ee.bin.blflash', 'rb') as fh:
        fh.seek(0x4000)
        check('4way flash file content', fh.read(256) == blob)
    check('4way keep alive', f.keep_alive())
    f.close()
    bl.stop()


def test_determinism(args):
    print('--- determinism')
    import sitl_fourway as fw

    def one_run():
        bl = Bootloader(args, ('--can-uri', 'none', '--line', 'high',
                               '--speedup', '5'))
        f = fw.FourWay(udp_port=args.port)
        time.sleep(0.3)
        stream = b''
        info = f.connect(timeout=4.0)
        stream += info or b''
        ee = f.read_flash(64, addr16=fw.ADDR_MAGIC_EEPROM)
        stream += ee or b''
        f.close()
        bl.stop()
        return stream

    a = one_run()
    b = one_run()
    check('deterministic reply streams', a == b and len(a) > 64,
          '%d bytes' % len(a))


def test_signals(args):
    print('--- signal detection')
    import sitl_dshot as sd
    # the 4way flash test overwrote the seeded app vectors; a fresh
    # flash file reseeds a bootable image (the eeprom stays programmed)
    if os.path.exists('bl_ee.bin.blflash'):
        os.remove('bl_ee.bin.blflash')

    def case(name, blargs, expect_exit, send=None, wait=5.0):
        bl = Bootloader(args, blargs)
        port = sd.InputPort('127.0.0.1', args.port)
        sender = None
        if send == 'dshot':
            sender = lambda: port.send_dshot(0, ptype=sd.TYPE_DSHOT600)
        elif send == 'low':
            sender = lambda: port.send_level(level=False)
        rc = bl.poll_exit(wait, send=sender)
        port.close()
        bl.stop()
        status = rc if rc is not None else 'stays'
        want = 42 if expect_exit else 'stays'
        check(name, status == want, 'got %s' % status)
        time.sleep(0.2)

    # eeprom is programmed by test_fourway (byte0=1, input_type=1)
    case('power-on line low boots', ('--can-uri', 'none', '--line', 'low'), True)
    case('watchdog line low boots', ('--can-uri', 'none', '--line', 'low',
                                     '--reset-cause', 'watchdog'), True)
    case('soft reset line low boots via float probe',
         ('--can-uri', 'none', '--line', 'low', '--reset-cause', 'software'), True)
    case('dshot stream boots', ('--can-uri', 'none'), True, send='dshot')
    case('delayed line low boots', ('--can-uri', 'none'), True, send='low')
    # 20ms of line low after entering the bootloader forces the jump
    bl = Bootloader(args, ('--can-uri', 'none', '--line', 'high'))
    port = sd.InputPort('127.0.0.1', args.port)
    time.sleep(0.5)
    rc = bl.poll_exit(5.0, send=lambda: port.send_level(level=False))
    port.close()
    bl.stop()
    check('20ms low forces jump', rc == 42, 'got %s' % rc)


def test_fourway_keeps_resident(args):
    '''an active 4-way session suppresses the no-CAN fallback boot'''
    print('--- 4way session keeps bootloader resident')
    import sitl_fourway as fw
    bl, f = fourway_session(args)
    ok = f.connect(timeout=4.0) is not None
    t0 = time.time()
    alive = True
    while time.time() - t0 < 3.0:
        f.keep_alive()
        if bl.proc.poll() is not None:
            alive = False
            break
        time.sleep(0.3)
    check('stays during 4way session', ok and alive)
    f.close()
    bl.stop()


def test_dronecan(args):
    print('--- dronecan')
    try:
        import dronecan
    except ImportError:
        print('SKIP: pydronecan not available')
        return
    import sitl_fourway as fw

    # mark the eeprom as a DroneCAN ESC with a fixed node id
    bl, f = fourway_session(args)
    assert f.connect(timeout=4.0) is not None
    ee = bytearray(f.read_flash(200, addr16=fw.ADDR_MAGIC_EEPROM))
    ee[0] = 0x01
    ee[46] = 5
    ee[176] = 25
    assert f.write(fw.ADDR_MAGIC_EEPROM, bytes(ee))
    f.close()
    bl.stop()

    bl = Bootloader(args, ('--can-uri', 'mcast:%d' % args.can_bus))
    node = dronecan.make_node('mcast:%d' % args.can_bus, node_id=100,
                              bitrate=1000000)
    seen = {}
    node.add_handler(dronecan.uavcan.protocol.NodeStatus,
                     lambda e: seen.__setitem__(e.transfer.source_node_id,
                                                e.message.mode))
    t0 = time.time()
    while time.time() - t0 < 8 and 25 not in seen:
        node.spin(0.1)
    check('dronecan nodestatus', 25 in seen and seen.get(25) == 2,
          str(seen))
    check('dronecan ESC waits for command', bl.proc.poll() is None)

    # firmware update over CAN
    image = make_image()
    with open('fwtest.bin', 'wb') as fh:
        fh.write(image)
    from dronecan.app.file_server import FileServer
    FileServer(node, lookup_paths=[os.getcwd()])
    req = dronecan.uavcan.protocol.file.BeginFirmwareUpdate.Request()
    req.image_file_remote_path.path = 'fwtest.bin'
    resp = {}
    node.request(req, 25, lambda e: resp.__setitem__('e', e))
    t0 = time.time()
    while time.time() - t0 < 5 and 'e' not in resp:
        node.spin(0.05)
    check('dronecan update accepted', resp.get('e') is not None
          and resp['e'].response.error == 0)
    cur = b''
    t0 = time.time()
    while time.time() - t0 < 60:
        node.spin(0.1)
        with open('bl_ee.bin.blflash', 'rb') as fh:
            fh.seek(0x4000)
            cur = fh.read(len(image))
        if cur == image:
            break
    check('dronecan update flash content', cur == image)

    # back on the bus after the update reset, then RawCommand boots
    seen.clear()
    t0 = time.time()
    while time.time() - t0 < 10 and 25 not in seen:
        node.spin(0.1)
    check('dronecan back after update', 25 in seen)
    rc = None
    t0 = time.time()
    while time.time() - t0 < 8:
        node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0, 0, 0, 0]))
        node.spin(0.05)
        rc = bl.proc.poll()
        if rc is not None:
            break
    check('dronecan rawcommand boots', rc == 42, 'got %s' % rc)
    node.close()
    bl.stop()


def test_chain(args):
    print('--- boot chain (execve)')
    import sitl_dshot as sd
    sys.path.insert(0, os.path.dirname(os.path.abspath(args.fw_tools + '/x')))
    from run_ci_tests import Sender, rpm_from_state
    from sitl_gui_backend import SimStream

    for f in ('chain_ee.bin', 'chain_ee.bin.blflash', 'chain_ee.bin.bkup',
              'chain_ee.bin.lock', 'chain.log'):
        if os.path.exists(f):
            os.remove(f)

    # seed the eeprom by running the fw once without a bootloader
    p = subprocess.Popen([args.app_elf, '--eeprom', 'chain_ee.bin',
                          '--can-uri', 'none', '--input-port', str(args.port),
                          '--state-port', str(args.state_port),
                          '--input-type', '1'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)
    p.kill()
    p.wait()

    def wait_log(needle, timeout=12.0, ofs=0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            data = open('chain.log', 'rb').read()[ofs:].decode(errors='replace')
            if needle in data:
                return True
            time.sleep(0.1)
        return False

    log = open('chain.log', 'wb')
    p = subprocess.Popen([args.app_elf, '--eeprom', 'chain_ee.bin',
                          '--can-uri', 'none', '--input-port', str(args.port),
                          '--state-port', str(args.state_port),
                          '--input-type', '1', '--bootloader', args.bootloader],
                         stdout=log, stderr=subprocess.STDOUT)
    snd = Sender(sd.TYPE_DSHOT600, bidir=True)
    snd.port = sd.InputPort('127.0.0.1', args.port)

    check('chain bootloader phase', wait_log('AM32 bootloader SITL'))
    check('chain jump to app', wait_log('SITL: jumping to application'))
    check('chain firmware runs', wait_log('AM32 SITL: eeprom='))

    sim = SimStream('127.0.0.1', args.state_port, period_us=200)
    sim.enabled = True
    time.sleep(3.0)
    snd.value = 800
    time.sleep(4.0)
    rpm = rpm_from_state(sim, 0.5)
    check('chain firmware spins', 3000 < rpm < 8000, 'rpm=%.0f' % rpm)

    ofs = os.path.getsize('chain.log')
    snd.stop()
    check('chain software reset', wait_log('SITL: reset (software)', 15.0, ofs))
    check('chain bootloader sees cause', wait_log('cause=1', 10.0, ofs))
    check('chain fallback boots again',
          wait_log('SITL: jumping to application', 15.0, ofs))
    sim.close()
    p.kill()
    p.wait()


def test_chain_ota(args):
    '''DroneCAN firmware update of a running firmware, handed off to the
    bootloader through the RTC backup registers across the exec chain'''
    print('--- boot chain OTA update')
    try:
        import dronecan
        from dronecan.app.file_server import FileServer
    except ImportError:
        print('SKIP: pydronecan not available')
        return
    import sitl_dshot as sd

    for f in ('ota_ee.bin', 'ota_ee.bin.blflash', 'ota_ee.bin.bkup',
              'ota_ee.bin.lock', 'ota.log'):
        if os.path.exists(f):
            os.remove(f)

    # seed eeprom (fw defaults) with a fixed node id
    p = subprocess.Popen([args.app_elf, '--eeprom', 'ota_ee.bin',
                          '--can-uri', 'none', '--input-port', str(args.port),
                          '--state-port', str(args.state_port),
                          '--input-type', '1', '--node-id', '27'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)
    p.kill()
    p.wait()

    # a dshot FC present from power-on: the pump runs before the chain
    # starts so the bootloader's early signal probe sees it, and fast
    # enough that frames land inside the ~5ms probe window
    port = sd.InputPort('127.0.0.1', args.port)
    import threading
    stop = threading.Event()

    def pump():
        while not stop.is_set():
            port.send_dshot(0, ptype=sd.TYPE_DSHOT600)
            time.sleep(0.0005)

    threading.Thread(target=pump, daemon=True).start()
    log = open('ota.log', 'wb')
    p = subprocess.Popen([args.app_elf, '--eeprom', 'ota_ee.bin',
                          '--can-uri', 'mcast:%d' % args.can_bus,
                          '--input-port', str(args.port),
                          '--state-port', str(args.state_port),
                          '--input-type', '1', '--bootloader', args.bootloader],
                         stdout=log, stderr=subprocess.STDOUT)

    image = make_image()
    with open('fw2.bin', 'wb') as fh:
        fh.write(image)
    node = dronecan.make_node('mcast:%d' % args.can_bus, node_id=100,
                              bitrate=1000000)
    FileServer(node, lookup_paths=[os.getcwd()])
    modes = {}
    node.add_handler(dronecan.uavcan.protocol.NodeStatus,
                     lambda e: modes.__setitem__(e.transfer.source_node_id,
                                                 e.message.mode))
    t0 = time.time()
    while time.time() - t0 < 20 and modes.get(27) != 0:
        node.spin(0.1)
    check('ota firmware on bus', modes.get(27) == 0, str(modes))

    # request the update; the fw resets into the bootloader instead of
    # replying (by design), so keep re-sending until the update runs
    req = dronecan.uavcan.protocol.file.BeginFirmwareUpdate.Request()
    req.image_file_remote_path.path = 'fw2.bin'
    cur = b''
    t0 = time.time()
    last_req = 0.0
    while time.time() - t0 < 90:
        if time.time() - last_req > 1.0:
            node.request(req, 27, lambda e: None)
            last_req = time.time()
        node.spin(0.1)
        if os.path.exists('ota_ee.bin.blflash'):
            with open('ota_ee.bin.blflash', 'rb') as fh:
                fh.seek(0x4000)
                cur = fh.read(len(image))
            if cur == image:
                break
    check('ota flash written by bootloader', cur == image)

    # the bootloader resets after the update; the dshot input boots the
    # firmware again
    modes.clear()
    t0 = time.time()
    while time.time() - t0 < 30 and modes.get(27) != 0:
        node.spin(0.1)
    check('ota firmware back after update', modes.get(27) == 0, str(modes))

    stop.set()
    node.close()
    port.close()
    p.kill()
    p.wait()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    default_bl = sorted(glob.glob(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', '..', '..', 'obj', 'AM32_SITL_BOOTLOADER_*_CAN_*.elf')))
    ap.add_argument('--bootloader', default=default_bl[0] if default_bl else None)
    ap.add_argument('--fw-tools', required=True,
                    help='path to Mcu/SITL in the am32-firmware repo')
    ap.add_argument('--app-elf', default=None,
                    help='AM32_SITL_CAN elf for the boot chain tests')
    ap.add_argument('--port', type=int, default=57920)
    ap.add_argument('--state-port', type=int, default=57921)
    ap.add_argument('--can-bus', type=int, default=8)
    args = ap.parse_args()
    if args.bootloader is None:
        print('no bootloader elf found, build AM32_SITL_BOOTLOADER_PB4_CAN')
        sys.exit(1)
    args.bootloader = os.path.abspath(args.bootloader)
    if args.app_elf:
        args.app_elf = os.path.abspath(args.app_elf)
    sys.path.insert(0, os.path.abspath(args.fw_tools))

    Bootloader(args, fresh=True).stop()  # clean state files

    test_fourway(args)
    test_determinism(args)
    test_signals(args)
    test_fourway_keeps_resident(args)
    test_dronecan(args)
    if args.app_elf:
        test_chain(args)
        test_chain_ota(args)
    else:
        print('SKIP: boot chain tests (no --app-elf)')

    if failures:
        print('\n%d FAILED: %s' % (len(failures), ', '.join(failures)))
        sys.exit(1)
    print('\nall bootloader tests passed')


if __name__ == '__main__':
    main()
