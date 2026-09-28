import csv

path = r'E:\ReckonX\test-runner\fixtures\reckonx_telemetry_log_2026-09-27T18-39-36.csv'
with open(path, 'r', encoding='utf-8') as f:
    lines = [line for line in f if not line.startswith('#')]
reader = csv.DictReader(lines)
rows = list(reader)

print(f"Total data rows: {len(rows)}")
print(f"Columns: {reader.fieldnames}")

def analyze_velocity_changes(target_range, label):
    print(f"\n========================================================")
    print(f"  {label} (Rows {target_range[0]} to {target_range[1]})")
    print(f"========================================================")
    for i in range(target_range[0], target_range[1] + 1):
        r = rows[i]
        prev = rows[i-1] if i > 0 else r
        vx = float(r['Raw_DR_VelX_ms'])
        vy = float(r['Raw_DR_VelY_ms'])
        pvx = float(prev['Raw_DR_VelX_ms'])
        pvy = float(prev['Raw_DR_VelY_ms'])
        dvx = vx - pvx
        dvy = vy - pvy
        dv = (dvx**2 + dvy**2)**0.5
        dt = (int(r['Timestamp_Epoch_Ms']) - int(prev['Timestamp_Epoch_Ms'])) / 1000.0 if i > 0 else 1.0
        accel = dv / dt if dt > 0 else 0
        has_gt = bool(r.get('GPS_GroundTruth_Lat'))
        print(f"Row {i:3d} | {r['Time_ISO'][11:19]} | {r['Source_Type']:15s} | dt={dt:.3f}s | spd={r['Speed_KmH']:6s} | vx={vx:8.4f} vy={vy:8.4f} | dv={dv:.4f} m/s | a_horiz={accel:.2f} m/s^2 | has_gt_gps={has_gt}")

analyze_velocity_changes((45, 55), "DR Rows 47-52")
analyze_velocity_changes((120, 132), "DR Rows 122-130")
analyze_velocity_changes((134, 140), "DR Rows 135-138")
analyze_velocity_changes((146, 154), "DR Rows 148-151")
analyze_velocity_changes((155, 166), "Escape Window Rows 156-164")

