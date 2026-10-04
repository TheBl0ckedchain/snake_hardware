#!/usr/bin/env python3
"""Read-only health check of the Dynamixel bus. It only pings and reads registers, so it never moves a motor.

Usage: python3 scripts/dxl_status.py [--port /dev/dynamixel] [--baud 1000000] [--scan-bauds]
"""
import argparse

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

# XM430-W350 control table (protocol 2.0): name -> (address, size)
REGS = {
    'model':        (0, 2),
    'firmware':     (6, 1),
    'baud_idx':     (8, 1),
    'return_delay': (9, 1),
    'op_mode':      (11, 1),
    'temp_limit':   (31, 1),
    'max_volt':     (32, 2),
    'min_volt':     (34, 2),
    'torque_on':    (64, 1),
    'hw_error':     (70, 1),
    'position':     (132, 4),
    'voltage':      (144, 2),
    'temperature':  (146, 1),
}
OP_MODES = {0: 'current', 1: 'velocity', 3: 'position', 4: 'ext-position', 5: 'current-pos', 16: 'pwm'}
HW_ERRORS = {0: 'input-voltage', 2: 'overheat', 3: 'encoder', 4: 'shock', 5: 'overload'}
BAUDS = [1000000, 57600, 115200, 2000000, 3000000, 4000000, 9600]


def read(ph, port, dxl_id, name):
    addr, size = REGS[name]
    fn = {1: ph.read1ByteTxRx, 2: ph.read2ByteTxRx, 4: ph.read4ByteTxRx}[size]
    val, res, err = fn(port, dxl_id, addr)
    if res != COMM_SUCCESS:
        raise IOError(f'id {dxl_id} {name}: {ph.getTxRxResult(res)}')
    if size == 4 and val >= 2**31:
        val -= 2**32
    return val


def scan(port, ph, baud):
    port.setBaudRate(baud)
    ids, res = ph.broadcastPing(port)
    return sorted(ids) if res == COMM_SUCCESS else []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/dynamixel')
    ap.add_argument('--baud', type=int, default=1000000)
    ap.add_argument('--scan-bauds', action='store_true', help='also try other baud rates if none answer')
    a = ap.parse_args()

    port, ph = PortHandler(a.port), PacketHandler(2.0)
    if not port.openPort():
        raise SystemExit(f'cannot open {a.port}')
    try:
        ids = scan(port, ph, a.baud)
        baud = a.baud
        if not ids and a.scan_bauds:
            for b in BAUDS:
                ids = scan(port, ph, b)
                if ids:
                    baud = b
                    break
        if not ids:
            raise SystemExit(f'no servos answered on {a.port} @ {a.baud} (power on? try --scan-bauds)')
        print(f'{len(ids)} servo(s) on {a.port} @ {baud}: ids {ids}\n')
        print(f"{'id':>3} {'model':>6} {'fw':>3} {'mode':>12} {'torque':>6} {'pos(deg)':>9} "
              f"{'volt':>5} {'temp':>4} {'rdelay':>6}  hw_error")
        for i in ids:
            r = {n: read(ph, port, i, n) for n in REGS}
            errs = [s for b, s in HW_ERRORS.items() if r['hw_error'] >> b & 1] or ['-']
            deg = (r['position'] - 2048) * 360.0 / 4096  # 2048 = joint centre
            print(f"{i:>3} {r['model']:>6} {r['firmware']:>3} {OP_MODES.get(r['op_mode'], r['op_mode']):>12} "
                  f"{'ON' if r['torque_on'] else 'off':>6} {deg:>9.1f} {r['voltage'] / 10:>5.1f} "
                  f"{r['temperature']:>4} {r['return_delay'] * 2:>4}us  {','.join(errs)}")
        missing = sorted(set(range(1, 7)) - set(ids))
        if missing:
            print(f'\nexpected ids 1-6, missing: {missing}')
    finally:
        port.closePort()


if __name__ == '__main__':
    main()
