import pandas as pd
import matplotlib.pyplot as plt
import sys
import tkinter as tk
from tkinter import filedialog
import numpy as np

# Initialize tkinter and hide the main window
root = tk.Tk()
root.withdraw()

# Pop up a native file browser dialog
print("Waiting for file selection...")
CSV_FILE = filedialog.askopenfilename(
    title="Select Telemetry CSV File",
    filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
)

# Clean up the tkinter instance
root.destroy()

if not CSV_FILE:
    print("Error: No file selected. Exiting.")
    sys.exit(1)

print(f"Loading data from: {CSV_FILE}")

try:
    df = pd.read_csv(CSV_FILE)
except Exception as e:
    print(f"Error loading {CSV_FILE}: {e}")
    sys.exit(1)

# Time proxy index
time = range(len(df))

# Create a large figure with 5 subplots
fig = plt.figure(figsize=(16, 12))
fig.canvas.manager.set_window_title('Telemetry Dashboard')

# ---------------------------------------------------------
# 1. Trajectory (XY) - Top View Line Plot (Bounded)
# ---------------------------------------------------------
ax1 = plt.subplot(3, 2, 1)
valid_gps = df[(df['gps_lat'] != 0) & (df['gps_long'] != 0)]
if not valid_gps.empty:
    ax1.plot(valid_gps['gps_long'], valid_gps['gps_lat'], '-', color='blue', linewidth=1.5, label='Flight Path')
    
    # Plot waypoint targets (ignoring 0,0)
    valid_wp = df[(df['waypoint_target_lat'] != 0) & (df['waypoint_target_lon'] != 0)]
    ax1.plot(valid_wp['waypoint_target_lon'], valid_wp['waypoint_target_lat'], 'rx', markersize=6, label='WP Targets')

    # Force the view to lock onto the actual GPS trajectory, ignoring garbage WP data
    lon_margin = 0.0005
    lat_margin = 0.0005
    ax1.set_xlim([valid_gps['gps_long'].min() - lon_margin, valid_gps['gps_long'].max() + lon_margin])
    ax1.set_ylim([valid_gps['gps_lat'].min() - lat_margin, valid_gps['gps_lat'].max() + lat_margin])

    # CRITICAL: Lock aspect ratio so the top-down view isn't stretched/distorted
    ax1.set_aspect('equal', adjustable='datalim')

ax1.set_title('1. Top View Trajectory (XY)')
ax1.set_xlabel('Longitude')
ax1.set_ylabel('Latitude')
ax1.legend()
ax1.grid(True, linestyle='--', alpha=0.6)

# ---------------------------------------------------------
# 2. Altitudes (Z axis) - OUTLIER FILTERED
# ---------------------------------------------------------
ax2 = plt.subplot(3, 2, 2)
ax2.plot(time, df['altitude'], label='Fused Alt', linewidth=2)
ax2.plot(time, df['des_altitude'], '--', label='Desired Alt')
ax2.plot(time, df['gps_alt'], label='GPS Alt', alpha=0.6)
ax2.plot(time, df['baro_altitude'], label='Baro Alt', alpha=0.6)
ax2.plot(time, df['waypoint_target_alt'], 'r:', label='WP Target Alt')

# --- Calculate dynamic visual limits to ignore boot spikes ---
# Filter out 0.0s and calculate the 5th and 95th percentiles of fused altitude
alt_data = df['altitude'][(df['altitude'] != 0) & (df['altitude'].notna())]
if not alt_data.empty:
    p5 = alt_data.quantile(0.05)
    p95 = alt_data.quantile(0.95)
    # Add a 20% vertical margin around the core data limits
    margin = (p95 - p5) * 0.2
    
    # If the variance is extremely small (e.g. flat on the ground), enforce a minimum view window
    if margin < 5: 
        margin = 10 
        
    ax2.set_ylim([p5 - margin, p95 + margin])

ax2.set_title('2. Altitudes (Outliers Visually Filtered)')
ax2.set_ylabel('Meters')
ax2.legend(loc='upper right', fontsize='small')
ax2.grid(True, linestyle='--', alpha=0.6)

# ---------------------------------------------------------
# 3. Attitude & Headings
# ---------------------------------------------------------
ax3 = plt.subplot(3, 2, 3)
ax3.plot(time, df['roll'], label='Roll')
ax3.plot(time, df['pitch'], label='Pitch')
ax3.plot(time, df['yaw'], label='Yaw')
ax3.plot(time, df['gps_heading'], '--', label='GPS Heading', alpha=0.6)
ax3.plot(time, df['waypoint_heading'], ':', label='WP Heading', alpha=0.6)

ax3.set_title('3. Attitude & Headings')
ax3.set_ylabel('Degrees')
ax3.legend(loc='upper right', fontsize='small')
ax3.grid(True, linestyle='--', alpha=0.6)

# ---------------------------------------------------------
# 4. Mission Progress & Speeds
# ---------------------------------------------------------
ax4 = plt.subplot(3, 2, 4)
ax4.plot(time, df['waypoint_distance'], label='Dist to WP (m)', color='purple')
ax4.plot(time, df['gps_speed'], label='GPS Speed (m/s)', color='orange')

ax4_twin = ax4.twinx()
ax4_twin.plot(time, df['waypoint_leg_progress'], 'g--', alpha=0.5, label='Leg Progress %')
ax4_twin.plot(time, df['waypoint_mission_progress'], 'b--', alpha=0.5, label='Mission Progress %')
ax4_twin.set_ylabel('Percentage (%)')

ax4.set_title('4. Mission Progress & Speed')
ax4.set_ylabel('Distance / Speed')
lines_1, labels_1 = ax4.get_legend_handles_labels()
lines_2, labels_2 = ax4_twin.get_legend_handles_labels()
ax4.legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper right', fontsize='small')
ax4.grid(True, linestyle='--', alpha=0.6)

# ---------------------------------------------------------
# 5. Discrete States: GPS Health, Flight Modes, WP Index
# ---------------------------------------------------------
ax5 = plt.subplot(3, 1, 3)
ax5.step(time, df['gps_sats'], label='GPS Sats', color='blue', where='post')
ax5.step(time, df['gps_fix_quality'], label='Fix Quality', color='cyan', where='post')
ax5.step(time, df['gps_lock_acquired'], label='Lock Acquired', color='green', where='post')

ax5.step(time, df['flightmode'] + 0.1, label='Flight Mode', color='red', where='post')
ax5.step(time, df['waypoint_index'] + 0.2, label='WP Index', color='purple', where='post')
ax5.step(time, df['waypoint_total'] + 0.3, '--', label='Total WPs', color='magenta', where='post')
ax5.step(time, df['waypoint_mission_complete'] + 0.4, label='Mission Complete', color='black', where='post')

ax5.set_title('5. Discrete States & GPS Health')
ax5.set_xlabel('Data Point Index (Time)')
ax5.set_ylabel('State / Count')
ax5.legend(loc='upper right', ncol=7, fontsize='small')
ax5.grid(True, linestyle='--', alpha=0.6)

plt.tight_layout()
plt.show()