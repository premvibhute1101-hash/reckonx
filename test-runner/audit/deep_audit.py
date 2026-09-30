import os
import difflib
import re

pairs = [
    ("EkfCore", "IDR_PRO/src/core/EkfCore.ts", "src/services/ekf/EkfCore.ts"),
    ("InsMechanization", "IDR_PRO/src/core/InsMechanization.ts", "src/services/ekf/InsMechanization.ts"),
    ("GnssQualityStateMachine", "IDR_PRO/src/core/GnssQualityStateMachine.ts", "src/services/ekf/GnssQualityStateMachine.ts"),
    ("OutputStabilizer", "IDR_PRO/src/core/OutputStabilizer.ts", "src/services/ekf/OutputStabilizer.ts"),
    ("AiMotionModel", "IDR_PRO/src/core/AiMotionModel.ts", "src/services/ekf/AiMotionModel.ts"),
    ("FusionRuntime", "IDR_PRO/src/core/FusionRuntime.ts", "src/services/ekf/FusionRuntime.ts"),
    ("SensorService", "IDR_PRO/src/services/SensorService.ts", "src/services/sensorService.ts"),
    ("DataLoggerService", "IDR_PRO/src/services/DataLoggerService.ts", "src/services/logExportService.ts"),
]

for name, idr_p, rx_p in pairs:
    if os.path.exists(idr_p) and os.path.exists(rx_p):
        with open(idr_p, 'r', encoding='utf-8') as f:
            idr_lines = f.readlines()
        with open(rx_p, 'r', encoding='utf-8') as f:
            rx_lines = f.readlines()
        print(f"=== {name} ===")
        print(f"IDR lines: {len(idr_lines)}, RX lines: {len(rx_lines)}")
        diff = list(difflib.unified_diff(idr_lines, rx_lines, fromfile=idr_p, tofile=rx_p))
        print(f"Diff lines count: {len(diff)}")
        out_diff = f"test-runner/audit/{name}.diff"
        with open(out_diff, 'w', encoding='utf-8') as df:
            df.writelines(diff)
        print(f"Saved diff to {out_diff}")
