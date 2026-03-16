#!/usr/bin/env python3
"""
Read live sensor data from STM32 StateEstimator via SWD memory reads.
Uses STM32_Programmer_CLI to read RAM through the ST-Link probe.
"""
import subprocess
import struct
import time
import sys

CLI = "/mnt/c/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe"

# StateEstimator base address (from ELF symbol table)
SE_BASE = 0x240037e0

# Member offsets (from GDB DWARF debug info)
OFFSETS = {
    # Estimated attitude (radians internally, we convert to degrees)
    "roll_rad":    2192,
    "pitch_rad":   2200,
    "yaw_rad":     2208,
    # IMU accel (m/s^2): ImuData(+2216) -> m_accel_mps2(+8) -> Vec3.m_x/y/z(+8/+16/+24)
    "accel_x":     2216 + 8 + 8,
    "accel_y":     2216 + 8 + 16,
    "accel_z":     2216 + 8 + 24,
    # IMU gyro (deg/s): ImuData(+2216) -> m_gyro_dps(+40) -> Vec3.m_x/y/z(+8/+16/+24)
    "gyro_x":      2216 + 40 + 8,
    "gyro_y":      2216 + 40 + 16,
    "gyro_z":      2216 + 40 + 24,
    # GPS: GpsData(+2296) -> lat(+8), lon(+16), alt(+24)
    "gps_lat":     2296 + 8,
    "gps_lon":     2296 + 16,
    "gps_alt":     2296 + 24,
    # Baro: BaroData(+2368) -> pressure(+8), altitude(+16)
    "baro_press":  2368 + 8,
    "baro_alt":    2368 + 16,
    # Mag: MagData(+2400) -> field_gauss.x/y/z(+8+8, +8+16, +8+24)
    "mag_x":       2400 + 8 + 8,
    "mag_y":       2400 + 8 + 16,
    "mag_z":       2400 + 8 + 24,
    # Position NED (m)
    "posN":        2448,
    "posE":        2456,
    "posD":        2464,
    # Velocity NED (m/s)
    "velN":        2472,
    "velE":        2480,
    "velD":        2488,
    # initialized flag
    "initialized": 2496,
}

import math
RAD2DEG = 180.0 / math.pi


def read_memory_block(addr, size_bytes):
    """Read a block of memory via STM32_Programmer_CLI, return raw bytes."""
    cmd = [CLI, "-c", "port=SWD", "mode=HotPlug", f"-r32", f"0x{addr:08X}", str(size_bytes)]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    output = result.stdout

    raw = bytearray()
    for line in output.splitlines():
        line = line.strip()
        # Lines look like: 0x24004070 : AABBCCDD EEFFGGHH ...
        if line.startswith("0x") and ":" in line:
            parts = line.split(":")[1].strip().split()
            for word_hex in parts:
                word_hex = word_hex.strip()
                if len(word_hex) == 8:
                    try:
                        word_val = int(word_hex, 16)
                        # STM32 is little-endian but programmer outputs big-endian words
                        raw.extend(struct.pack("<I", word_val))
                    except ValueError:
                        pass
    return bytes(raw)


def extract_double(block, base_addr, field_offset):
    """Extract a little-endian double from a memory block."""
    abs_addr = SE_BASE + field_offset
    local_offset = abs_addr - base_addr
    if local_offset < 0 or local_offset + 8 > len(block):
        return float('nan')
    return struct.unpack_from("<d", block, local_offset)[0]


def read_all_sensors():
    """Read all sensor data in one SWD transaction."""
    # Read from roll (offset 2192) through initialized (offset 2496+8)
    start_addr = SE_BASE + 2192
    end_addr = SE_BASE + 2504
    size = end_addr - start_addr

    block = read_memory_block(start_addr, size)
    if len(block) < size:
        return None

    data = {}
    for name, offset in OFFSETS.items():
        if name == "initialized":
            abs_addr = SE_BASE + offset
            local_off = abs_addr - start_addr
            if local_off >= 0 and local_off < len(block):
                data[name] = block[local_off]
            else:
                data[name] = 0
        else:
            data[name] = extract_double(block, start_addr, offset)
    return data


def read_imu_block():
    """Read IMU data separately (it's before the roll offset)."""
    start_addr = SE_BASE + 2216 + 8 + 8  # accel_x
    end_addr = SE_BASE + 2216 + 40 + 24 + 8  # gyro_z end
    size = end_addr - start_addr
    block = read_memory_block(start_addr, size)
    if len(block) < size:
        return {}
    result = {}
    for name in ["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]:
        result[name] = extract_double(block, start_addr, OFFSETS[name])
    return result


def main():
    n_samples = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    print(f"Reading {n_samples} samples at {interval}s intervals...")
    print(f"StateEstimator @ 0x{SE_BASE:08X}\n")

    samples = []
    for i in range(n_samples):
        t0 = time.time()
        data = read_all_sensors()
        imu = read_imu_block()
        elapsed = time.time() - t0

        if data is None:
            print(f"[Sample {i+1}] FAILED to read memory")
            continue

        data.update(imu)
        data["_time"] = time.time()
        data["_read_ms"] = elapsed * 1000
        samples.append(data)

        print(f"[Sample {i+1}/{n_samples}] read in {elapsed*1000:.0f}ms")
        if i < n_samples - 1:
            time.sleep(max(0, interval - elapsed))

    # Print results table
    print("\n" + "=" * 120)
    print("SENSOR DATA TABLE")
    print("=" * 120)

    # Attitude
    print(f"\n{'ATTITUDE (deg)':^60}")
    print(f"{'Sample':>8} | {'Roll':>10} | {'Pitch':>10} | {'Yaw':>10} | {'Init':>5}")
    print("-" * 60)
    for i, s in enumerate(samples):
        print(f"{i+1:>8} | {s.get('roll_rad',0)*RAD2DEG:>10.3f} | {s.get('pitch_rad',0)*RAD2DEG:>10.3f} | {s.get('yaw_rad',0)*RAD2DEG:>10.3f} | {s.get('initialized',0):>5}")

    # IMU
    print(f"\n{'IMU ACCELEROMETER (m/s2)':^60}")
    print(f"{'Sample':>8} | {'Accel X':>10} | {'Accel Y':>10} | {'Accel Z':>10}")
    print("-" * 60)
    for i, s in enumerate(samples):
        print(f"{i+1:>8} | {s.get('accel_x',0):>10.3f} | {s.get('accel_y',0):>10.3f} | {s.get('accel_z',0):>10.3f}")

    print(f"\n{'IMU GYROSCOPE (deg/s)':^60}")
    print(f"{'Sample':>8} | {'Gyro X':>10} | {'Gyro Y':>10} | {'Gyro Z':>10}")
    print("-" * 60)
    for i, s in enumerate(samples):
        print(f"{i+1:>8} | {s.get('gyro_x',0):>10.3f} | {s.get('gyro_y',0):>10.3f} | {s.get('gyro_z',0):>10.3f}")

    # GPS
    print(f"\n{'GPS':^80}")
    print(f"{'Sample':>8} | {'Latitude':>14} | {'Longitude':>14} | {'Alt MSL (m)':>12}")
    print("-" * 60)
    for i, s in enumerate(samples):
        print(f"{i+1:>8} | {s.get('gps_lat',0):>14.7f} | {s.get('gps_lon',0):>14.7f} | {s.get('gps_alt',0):>12.2f}")

    # Baro
    print(f"\n{'BAROMETER':^60}")
    print(f"{'Sample':>8} | {'Pressure (Pa)':>14} | {'Altitude (m)':>14}")
    print("-" * 60)
    for i, s in enumerate(samples):
        print(f"{i+1:>8} | {s.get('baro_press',0):>14.1f} | {s.get('baro_alt',0):>14.2f}")

    # Mag
    print(f"\n{'MAGNETOMETER (gauss)':^60}")
    print(f"{'Sample':>8} | {'Mag X':>10} | {'Mag Y':>10} | {'Mag Z':>10}")
    print("-" * 60)
    for i, s in enumerate(samples):
        print(f"{i+1:>8} | {s.get('mag_x',0):>10.4f} | {s.get('mag_y',0):>10.4f} | {s.get('mag_z',0):>10.4f}")

    # Position & Velocity NED
    print(f"\n{'POSITION NED (m) & VELOCITY NED (m/s)':^80}")
    print(f"{'Sample':>8} | {'PosN':>10} | {'PosE':>10} | {'PosD':>10} | {'VelN':>8} | {'VelE':>8} | {'VelD':>8}")
    print("-" * 80)
    for i, s in enumerate(samples):
        print(f"{i+1:>8} | {s.get('posN',0):>10.2f} | {s.get('posE',0):>10.2f} | {s.get('posD',0):>10.2f} | {s.get('velN',0):>8.3f} | {s.get('velE',0):>8.3f} | {s.get('velD',0):>8.3f}")

    print("\n" + "=" * 120)


if __name__ == "__main__":
    main()
