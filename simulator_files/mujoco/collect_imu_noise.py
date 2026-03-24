#!/usr/bin/env python3
"""
Collect accelerometer and gyroscope data from a real Crazyflie to compute
sensor model parameters: scale, bias, and noise density.

Place the Crazyflie on a flat, still surface (Z-up) before running.
For Allan variance (bias walk), use at least 300s.

Usage:
    python3 collect_imu_noise.py [URI] [DURATION_S]
    python3 collect_imu_noise.py radio://0/50/2M/E7E7E7E7E0 300

Output: scale, bias, noise density per axis for both accel and gyro.
"""
import sys
import time

import cflib.crtp
import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

URI = sys.argv[1] if len(sys.argv) > 1 else 'radio://0/80/2M/E7E7E7E7E7'
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
GRAVITY = 9.81  # m/s²


def allan_variance(data, dt):
    """Compute Allan deviation for a 1D time series."""
    n = len(data)
    max_clusters = n // 2
    cluster_sizes = np.unique(np.logspace(0, np.log10(max_clusters), 100).astype(int))
    cluster_sizes = cluster_sizes[(cluster_sizes >= 1) & (cluster_sizes <= max_clusters)]

    taus, adevs = [], []
    for m in cluster_sizes:
        tau = m * dt
        n_use = (n // m) * m
        if n_use < 2 * m:
            continue
        clusters = data[:n_use].reshape(-1, m).mean(axis=1)
        avar = 0.5 * np.mean(np.diff(clusters) ** 2)
        taus.append(tau)
        adevs.append(np.sqrt(avar))
    return np.array(taus), np.array(adevs)


def extract_allan_params(taus, adevs):
    """Extract noise density, bias instability, random walk from Allan dev."""
    # Noise density: adev * sqrt(tau) at tau closest to 1s
    idx_1s = np.argmin(np.abs(taus - 1.0))
    noise_density = adevs[idx_1s] * np.sqrt(taus[idx_1s])

    # Bias instability: minimum of curve
    idx_min = np.argmin(adevs)
    bias_instability = adevs[idx_min]
    tau_bi = taus[idx_min]

    # Rate random walk: adev * sqrt(3/tau) at longest tau
    rate_random_walk = adevs[-1] * np.sqrt(3.0 / taus[-1]) if len(taus) > 5 else float('nan')

    return noise_density, bias_instability, tau_bi, rate_random_walk


def main():
    cflib.crtp.init_drivers()

    print(f'Connecting to {URI} ...')
    print('Place Crazyflie FLAT on a still surface (Z-up)!')
    if DURATION < 60:
        print(f'Note: {DURATION}s is short for Allan variance. Use 300s+ for bias walk.')
    print()

    acc_data = []
    gyro_data = []

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        log_acc = LogConfig(name='acc', period_in_ms=10)
        log_acc.add_variable('acc.x', 'float')
        log_acc.add_variable('acc.y', 'float')
        log_acc.add_variable('acc.z', 'float')

        log_gyro = LogConfig(name='gyro', period_in_ms=10)
        log_gyro.add_variable('gyro.x', 'float')
        log_gyro.add_variable('gyro.y', 'float')
        log_gyro.add_variable('gyro.z', 'float')

        print(f'Collecting {DURATION}s of data at 100Hz ...')
        t0 = time.time()

        with SyncLogger(scf, [log_acc, log_gyro]) as logger:
            for timestamp, data, logconfig in logger:
                if logconfig.name == 'acc':
                    acc_data.append([data['acc.x'], data['acc.y'], data['acc.z']])
                elif logconfig.name == 'gyro':
                    gyro_data.append([data['gyro.x'], data['gyro.y'], data['gyro.z']])

                elapsed = time.time() - t0
                if len(acc_data) % 500 == 0:
                    print(f'  {elapsed:.0f}s / {DURATION:.0f}s  '
                          f'({len(acc_data)} samples)', end='\r')
                if elapsed >= DURATION:
                    break

    print()
    acc = np.array(acc_data)
    gyro = np.array(gyro_data)
    acc_rate = len(acc) / DURATION
    gyro_rate = len(gyro) / DURATION
    acc_dt = 1.0 / acc_rate
    gyro_dt = 1.0 / gyro_rate

    print(f'Collected {len(acc)} accel samples ({acc_rate:.1f} Hz), '
          f'{len(gyro)} gyro samples ({gyro_rate:.1f} Hz)')
    print()

    # =====================================================================
    # Check if data is in g or m/s²
    # =====================================================================
    acc_mean = acc.mean(axis=0)
    z_magnitude = abs(acc_mean[2])
    if z_magnitude < 2.0:
        # Data is in g, convert to m/s²
        print(f'  Detected accel units: g (Z mean = {acc_mean[2]:.4f})')
        acc = acc * GRAVITY
        acc_mean = acc.mean(axis=0)
        in_g = True
    else:
        print(f'  Detected accel units: m/s² (Z mean = {acc_mean[2]:.4f})')
        in_g = False

    # =====================================================================
    # ACCELEROMETER: scale, bias, noise
    # =====================================================================
    # With CF flat (Z-up), true values are [0, 0, g].
    # output = scale * true + bias + noise
    #
    # X: mean_x = scale_x * 0 + bias_x → bias_x = mean_x
    # Y: mean_y = scale_y * 0 + bias_y → bias_y = mean_y
    # Z: mean_z = scale_z * g + bias_z
    #    We can't separate scale_z and bias_z from one orientation.
    #    Assume bias_z ≈ same magnitude as bias_x/y, then:
    #    scale_z = (mean_z - bias_z_est) / g
    #    For a first approximation: scale_z = mean_z / g

    acc_std = acc.std(axis=0)
    acc_nd = acc_std / np.sqrt(acc_rate)

    acc_bias = np.array([acc_mean[0], acc_mean[1], 0.0])  # can't determine Z bias from one pose
    acc_scale = np.array([1.0, 1.0, acc_mean[2] / GRAVITY])  # X/Y scale needs multi-pose

    print('=== ACCELEROMETER ===')
    print(f'  Mean:    X={acc_mean[0]:+.6f}  Y={acc_mean[1]:+.6f}  Z={acc_mean[2]:+.6f} m/s²')
    print()
    print(f'  Bias:    X={acc_bias[0]:+.6f}  Y={acc_bias[1]:+.6f}  Z=? (need flipped data) m/s²')
    print(f'           X={acc_bias[0]/GRAVITY*1000:+.3f}  Y={acc_bias[1]/GRAVITY*1000:+.3f} mg')
    print()
    print(f'  Scale:   X=? (need multi-axis)  Y=? (need multi-axis)  Z={acc_scale[2]:.6f}')
    print(f'           Z error: {(acc_scale[2] - 1.0) * 100:+.4f}%')
    print()
    print(f'  Noise density (m/s²/√Hz):')
    print(f'           X={acc_nd[0]:.6f}  Y={acc_nd[1]:.6f}  Z={acc_nd[2]:.6f}')
    acc_nd_ug = acc_nd / GRAVITY * 1e6
    print(f'  Noise density (µg/√Hz):')
    print(f'           X={acc_nd_ug[0]:.1f}  Y={acc_nd_ug[1]:.1f}  Z={acc_nd_ug[2]:.1f}')
    print()

    # =====================================================================
    # GYROSCOPE: scale, bias, noise
    # =====================================================================
    # Stationary: true = [0, 0, 0] °/s
    # output = scale * 0 + bias + noise → mean = bias
    # Scale needs a known rotation rate (turntable) — can't determine here.

    gyro_mean = gyro.mean(axis=0)
    gyro_std = gyro.std(axis=0)
    gyro_nd_dps = gyro_std / np.sqrt(gyro_rate)
    gyro_nd_rps = gyro_nd_dps * np.pi / 180.0

    print('=== GYROSCOPE ===')
    print(f'  Bias (°/s):  X={gyro_mean[0]:+.6f}  Y={gyro_mean[1]:+.6f}  Z={gyro_mean[2]:+.6f}')
    print(f'  Bias (rad/s): X={gyro_mean[0]*np.pi/180:+.8f}  Y={gyro_mean[1]*np.pi/180:+.8f}  Z={gyro_mean[2]*np.pi/180:+.8f}')
    print(f'  Scale:       (need known rotation rate — turntable test)')
    print()
    print(f'  Noise density (°/s/√Hz):')
    print(f'           X={gyro_nd_dps[0]:.6f}  Y={gyro_nd_dps[1]:.6f}  Z={gyro_nd_dps[2]:.6f}')
    print(f'  Noise density (rad/s/√Hz):')
    print(f'           X={gyro_nd_rps[0]:.8f}  Y={gyro_nd_rps[1]:.8f}  Z={gyro_nd_rps[2]:.8f}')
    print()

    # =====================================================================
    # ALLAN VARIANCE (if enough data)
    # =====================================================================
    if DURATION >= 30:
        print('=== ALLAN VARIANCE ===')
        for name, data_arr, data_dt, unit, to_rad in [
            ('Accel', acc, acc_dt, 'm/s²', 1.0),
            ('Gyro', gyro, gyro_dt, '°/s', np.pi / 180.0),
        ]:
            print(f'  --- {name} ---')
            for ax, label in enumerate(['X', 'Y', 'Z']):
                taus, adevs = allan_variance(data_arr[:, ax], data_dt)
                if len(taus) < 3:
                    print(f'    {label}: not enough data')
                    continue
                nd, bi, tau_bi, rrw = extract_allan_params(taus, adevs)
                print(f'    {label}: noise_density={nd:.6f} {unit}/√Hz  '
                      f'bias_instab={bi:.6f} {unit} (τ={tau_bi:.1f}s)  '
                      f'random_walk={rrw:.6f} {unit}/√s')
                if name == 'Gyro':
                    print(f'         → random_walk={rrw * to_rad:.8f} rad/s/√s')
                if name == 'Accel':
                    print(f'         → random_walk={rrw:.8f} m/s²/√s')
            print()
    else:
        print('(Skipping Allan variance — need ≥30s of data)')
        print()

    # =====================================================================
    # SUMMARY for crazysim.py
    # =====================================================================
    print('=== SUGGESTED crazysim.py PARAMETERS ===')
    print('_BMI088_NOISE = {')
    print(f"    'acc_noise_density': [{acc_nd[0]:.6f}, {acc_nd[1]:.6f}, {acc_nd[2]:.6f}],")
    print(f"    'gyro_noise_density': [{gyro_nd_rps[0]:.6f}, {gyro_nd_rps[1]:.6f}, {gyro_nd_rps[2]:.6f}],")
    if DURATION >= 30:
        acc_rrws = []
        gyro_rrws = []
        for ax in range(3):
            taus, adevs = allan_variance(acc[:, ax], acc_dt)
            _, _, _, rrw = extract_allan_params(taus, adevs)
            acc_rrws.append(rrw)
            taus, adevs = allan_variance(gyro[:, ax], gyro_dt)
            _, _, _, rrw = extract_allan_params(taus, adevs)
            gyro_rrws.append(rrw * np.pi / 180.0)
        print(f"    'acc_bias_walk': {np.mean(acc_rrws):.6f},")
        print(f"    'gyro_bias_walk': {np.mean(gyro_rrws):.8f},")
    else:
        print(f"    'acc_bias_walk': ???,  # run with 300s for Allan variance")
        print(f"    'gyro_bias_walk': ???,  # run with 300s for Allan variance")
    print(f"    'baro_noise_std': 0.1,")
    print('}')
    print()

    print(f'Accel bias (for sensor model):')
    print(f"    'acc_bias': [{acc_bias[0]:.6f}, {acc_bias[1]:.6f}, 0.0],  # m/s² [X,Y,Z]")
    print(f'Gyro bias (for sensor model):')
    gb = gyro_mean * np.pi / 180.0
    print(f"    'gyro_bias': [{gb[0]:.8f}, {gb[1]:.8f}, {gb[2]:.8f}],  # rad/s [X,Y,Z]")
    print()

    # Save raw data
    outfile = 'imu_noise_data.csv'
    with open(outfile, 'w') as f:
        f.write('acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n')
        n = min(len(acc), len(gyro))
        for i in range(n):
            f.write(f'{acc[i,0]},{acc[i,1]},{acc[i,2]},'
                    f'{gyro[i,0]},{gyro[i,1]},{gyro[i,2]}\n')
    print(f'Raw data saved to {outfile}')

    # Datasheet comparison
    print()
    print('=== BMI088 DATASHEET COMPARISON ===')
    print(f'  Accel noise (datasheet): 160/160/190 µg/√Hz  |  measured: {acc_nd_ug[0]:.0f}/{acc_nd_ug[1]:.0f}/{acc_nd_ug[2]:.0f} µg/√Hz')
    print(f'  Gyro noise (datasheet):  0.014 °/s/√Hz       |  measured: {gyro_nd_dps[0]:.4f}/{gyro_nd_dps[1]:.4f}/{gyro_nd_dps[2]:.4f} °/s/√Hz')
    print(f'  Accel offset (datasheet): ±20 mg              |  measured: {acc_bias[0]/GRAVITY*1000:+.2f}/{acc_bias[1]/GRAVITY*1000:+.2f} mg')
    print(f'  Gyro offset (datasheet):  ±1 °/s              |  measured: {gyro_mean[0]:+.4f}/{gyro_mean[1]:+.4f}/{gyro_mean[2]:+.4f} °/s')


if __name__ == '__main__':
    main()
