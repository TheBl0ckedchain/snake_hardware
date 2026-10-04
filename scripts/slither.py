"""
Snake Robot Slithering Controller
==================================
Drives 6 Dynamixel XM430-W350 servos in a sinusoidal traveling-wave
pattern to produce a slow, smooth slithering (lateral undulation) gait.

Requirements:
    pip install dynamixel-sdk

Wiring / setup assumptions:
    - All 6 motors are daisy-chained on the same TTL bus (U2D2 or similar).
    - Motor IDs are 1..6 (change MOTOR_IDS below if different).
    - Each motor is in Position Control Mode (operating mode 3), which is
      the XM430 default.
    - Baudrate 1 Mbps, set once by scripts/dxl_configure.py.

Tune AMPLITUDE_DEG, WAVELENGTH, PERIOD_S, and DIRECTION to change the gait.
"""

import time
import math
from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_LOWORD,
    DXL_HIWORD,
)

# ----------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------

DEVICENAME = "/dev/dynamixel"   # udev name (see udev/); "COM3" etc. on Windows
BAUDRATE = 1000000                # set by scripts/dxl_configure.py (factory default 57600)
PROTOCOL_VERSION = 2.0

MOTOR_IDS = [1, 2, 3, 4, 5, 6]   # order = head to tail along the body

# Control table addresses (XM430-W350, Protocol 2.0)
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
POSITION_CONTROL_MODE = 3

# XM430 position units: 0-4095 ticks = 0-360 degrees
TICKS_PER_REV = 4096
CENTER_TICKS = 2048              # "straight" position for each joint

# ----------------------------------------------------------------------
# Gait parameters
# ----------------------------------------------------------------------

AMPLITUDE_DEG = 25.0    # how far each joint swings left/right from center
PERIOD_S = 4.0          # seconds for one full wave cycle (slow = larger)
WAVELENGTH_JOINTS = 6.0 # how many joints span one full sine wavelength
DIRECTION = 1           # 1 = wave travels head->tail (forward), -1 = reverse
UPDATE_RATE_HZ = 20     # how often we send new goal positions

AMPLITUDE_TICKS = int(AMPLITUDE_DEG / 360.0 * TICKS_PER_REV)


def deg_to_ticks(deg):
    return int(deg / 360.0 * TICKS_PER_REV)


def main():
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        raise IOError(f"Failed to open port {DEVICENAME}")
    if not port_handler.setBaudRate(BAUDRATE):
        raise IOError(f"Failed to set baudrate {BAUDRATE}")

    print("Port opened successfully.")

    # --- Set operating mode to Position Control, then enable torque ---
    for motor_id in MOTOR_IDS:
        # Torque must be off to change operating mode
        packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        result, error = packet_handler.write1ByteTxRx(
            port_handler, motor_id, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE
        )
        if result != COMM_SUCCESS:
            print(f"[WARN] Motor {motor_id}: failed to set operating mode: "
                  f"{packet_handler.getTxRxResult(result)}")

        result, error = packet_handler.write1ByteTxRx(
            port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if result != COMM_SUCCESS:
            print(f"[WARN] Motor {motor_id}: failed to enable torque: "
                  f"{packet_handler.getTxRxResult(result)}")
        else:
            print(f"Motor {motor_id}: torque enabled, position mode set.")

    group_sync_write = GroupSyncWrite(
        port_handler, packet_handler, ADDR_GOAL_POSITION, 4
    )

    print("Starting slither motion. Press Ctrl+C to stop.")

    start_time = time.time()
    dt = 1.0 / UPDATE_RATE_HZ

    try:
        while True:
            t = time.time() - start_time
            group_sync_write.clearParam()

            for i, motor_id in enumerate(MOTOR_IDS):
                # Traveling wave: phase shifts along the body by joint index
                phase = 2 * math.pi * (t / PERIOD_S - DIRECTION * i / WAVELENGTH_JOINTS)
                offset_ticks = int(AMPLITUDE_TICKS * math.sin(phase))
                goal_position = CENTER_TICKS + offset_ticks

                # Clamp to valid range just in case amplitude is set too high
                goal_position = max(0, min(TICKS_PER_REV - 1, goal_position))

                param_goal_position = [
                    DXL_LOBYTE(DXL_LOWORD(goal_position)),
                    DXL_HIBYTE(DXL_LOWORD(goal_position)),
                    DXL_LOBYTE(DXL_HIWORD(goal_position)),
                    DXL_HIBYTE(DXL_HIWORD(goal_position)),
                ]
                if not group_sync_write.addParam(motor_id, param_goal_position):
                    print(f"[WARN] Motor {motor_id}: failed to add sync write param")

            comm_result = group_sync_write.txPacket()
            if comm_result != COMM_SUCCESS:
                print(f"[WARN] Sync write failed: {packet_handler.getTxRxResult(comm_result)}")

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nStopping motion.")

    finally:
        # Disable torque on all motors before exiting
        for motor_id in MOTOR_IDS:
            packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        port_handler.closePort()
        print("Port closed. Motors torque-disabled.")


if __name__ == "__main__":
    main()
