#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
import sys

def visualize_trajectory(csv_file, target_points=50000):

    df_csv = pd.read_csv(csv_file)
    print(f'[INFO] Loaded {len(df_csv)} trajectory points from {csv_file}.')

    step = max(1, len(df_csv) // target_points)
    if step > 1:
        df = df_csv.iloc[::step].reset_index(drop=True)
        print(f'[INFO] Downsampled {len(df_csv)} points to {len(df)} points for plotting.')
    else:
        df = df_csv
        print(f'[INFO] No downsampling required.')

    fig = plt.figure(figsize=(18, 6))
    fig.canvas.manager.set_window_title(f'Trajectory Analysis: {csv_file}')

    ax1 = fig.add_subplot(1, 3, 1, projection='3d')
    ax1.plot(df['pos.x'], df['pos.y'], df['pos.z'], color='blue', linewidth=2)
    ax1.set_title('3D Tool Path')
    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_zlabel('Z (mm)')

    ax2 = fig.add_subplot(1, 3, 2)
    ax2.plot(df['t'], df['vel.x'], label='Vel X', color='red', alpha=0.8)
    ax2.plot(df['t'], df['vel.y'], label='Vel Y', color='green', alpha=0.8)
    ax2.plot(df['t'], df['vel.z'], label='Vel Z', color='blue', alpha=0.8)
    ax2.set_title('Velocity vs Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (mm/s)')
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.6)

    ax3 = fig.add_subplot(1, 3, 3)
    ax3.plot(df['t'], df['acc.x'], label='Acc X', color='red', alpha=0.8)
    ax3.plot(df['t'], df['acc.y'], label='Acc Y', color='green', alpha=0.8)
    ax3.plot(df['t'], df['acc.z'], label='Acc Z', color='blue', alpha=0.8)
    ax3.set_title('Acceleration vs Time')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Acceleration (mm/s²)')
    ax3.legend()
    ax3.grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(f'[ERROR] Usage: {sys.argv[0]} <path_to_trajectory.csv>')
    else:
        visualize_trajectory(sys.argv[1])