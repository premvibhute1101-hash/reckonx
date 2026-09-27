"""
run_all_tests.py
================
Master test orchestrator for ReckonX Full Test Suite.
Executes:
1. Python Unit & Component Tests (15-State EKF, HMM Map Matching, INS Mechanization, Seamless Controller)
2. Frontend & Core TypeScript Unit Tests (InsMechanization, GnssQualityStateMachine, OutputStabilizer, DistanceFormatter, RouteProgress)
3. Backend REST API Integration Tests (Fastify routes, in-memory DB, Session Ingestion, Telemetry Sync)
4. End-to-End Mission Pipeline Test (Full sensor -> INS -> AI -> 15-State EKF -> HMM Map Match lifecycle)

Outputs:
- Live console telemetry
- test-output/test_report.json
- test-output/test_report.md
- test-output/test_report.html
"""

import os
import sys
import subprocess
import time
import json
from datetime import datetime

# Ensure UTF-8 output where supported, or graceful ASCII fallback
if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUTPUT_DIR = os.path.join(ROOT_DIR, "test-output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

test_suites_summary = []


def run_python_suite(name: str, script_path: str, category: str):
    print(f"\n>> Running [{category}] {name}...")
    start_time = time.time()
    res = subprocess.run(
        [sys.executable, script_path],
        cwd=ROOT_DIR,
        capture_output=True,
        text=True
    )
    duration_s = time.time() - start_time
    passed = (res.returncode == 0)

    # Parse test count and output
    output_lines = (res.stdout + "\n" + res.stderr).strip().splitlines()
    for line in output_lines:
        if "Ran " in line and " tests in " in line:
            print(f"   {line}")
        elif line.startswith("OK") or line.startswith("FAILED"):
            print(f"   Status: {line}")

    test_suites_summary.append({
        "name": name,
        "category": category,
        "passed": passed,
        "duration_s": round(duration_s, 3),
        "output": "\n".join(output_lines),
        "returncode": res.returncode
    })
    status_icon = "[PASS]" if passed else "[FAIL]"
    print(f"   Result: {status_icon} ({duration_s:.3f}s)")


def run_node_ts_suite(name: str, script_path: str, category: str):
    print(f"\n>> Running [{category}] {name}...")
    start_time = time.time()
    # Execute with npx tsx
    cmd = f"npx tsx {script_path}"
    res = subprocess.run(
        cmd,
        shell=True,
        cwd=os.path.join(ROOT_DIR, "backend"),
        capture_output=True,
        text=True
    )
    duration_s = time.time() - start_time
    passed = (res.returncode == 0)

    output_lines = (res.stdout + "\n" + res.stderr).strip().splitlines()
    for line in output_lines:
        if line.startswith("  [PASS]") or line.startswith("  [FAIL]") or "Passed" in line:
            print(f"   {line}")

    test_suites_summary.append({
        "name": name,
        "category": category,
        "passed": passed,
        "duration_s": round(duration_s, 3),
        "output": "\n".join(output_lines),
        "returncode": res.returncode
    })
    status_icon = "[PASS]" if passed else "[FAIL]"
    print(f"   Result: {status_icon} ({duration_s:.3f}s)")


def generate_reports():
    total_suites = len(test_suites_summary)
    passed_suites = sum(1 for s in test_suites_summary if s["passed"])
    failed_suites = total_suites - passed_suites
    total_duration = sum(s["duration_s"] for s in test_suites_summary)

    # 1. JSON Report
    json_path = os.path.join(OUTPUT_DIR, "test_report.json")
    report_data = {
        "timestamp": datetime.now().isoformat(),
        "summary": {
            "total_suites": total_suites,
            "passed": passed_suites,
            "failed": failed_suites,
            "duration_s": round(total_duration, 3),
            "status": "PASSED" if failed_suites == 0 else "FAILED"
        },
        "suites": test_suites_summary
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(report_data, f, indent=2)

    # 2. Markdown Report
    md_path = os.path.join(OUTPUT_DIR, "test_report.md")
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# ReckonX Comprehensive Test Suite Report\n\n")
        f.write(f"- **Execution Time:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"- **Overall Status:** {'✅ ALL TESTS PASSED' if failed_suites == 0 else '❌ TESTS FAILED'}\n")
        f.write(f"- **Total Test Suites:** {total_suites}\n")
        f.write(f"- **Passed:** {passed_suites} | **Failed:** {failed_suites}\n")
        f.write(f"- **Total Runtime:** {total_duration:.2f} seconds\n\n")
        f.write("## Test Suite Breakdown\n\n")
        f.write("| Suite Name | Level / Category | Status | Runtime (s) |\n")
        f.write("| :--- | :--- | :---: | :---: |\n")
        for s in test_suites_summary:
            status = "✅ PASS" if s["passed"] else "❌ FAIL"
            f.write(f"| {s['name']} | {s['category']} | {status} | {s['duration_s']}s |\n")

    # 3. Self-contained HTML Report
    html_path = os.path.join(OUTPUT_DIR, "test_report.html")
    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>ReckonX Test Suite Report</title>
  <style>
    body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; background: #0F172A; color: #F8FAFC; margin: 0; padding: 40px 20px; }}
    .container {{ max-width: 960px; margin: 0 auto; background: #1E293B; border: 1px solid #334155; border-radius: 12px; padding: 32px; box-shadow: 0 10px 25px rgba(0,0,0,0.5); }}
    h1 {{ margin-top: 0; font-size: 24px; color: #38BDF8; }}
    .badge {{ display: inline-block; padding: 6px 14px; border-radius: 9999px; font-weight: bold; font-size: 13px; text-transform: uppercase; }}
    .badge-pass {{ background: #065F46; color: #34D399; border: 1px solid #059669; }}
    .badge-fail {{ background: #7F1D1D; color: #F87171; border: 1px solid #DC2626; }}
    .stats-grid {{ display: grid; grid-template-columns: repeat(4, 1fr); gap: 16px; margin: 24px 0; }}
    .stat-card {{ background: #0F172A; border: 1px solid #334155; border-radius: 8px; padding: 16px; text-align: center; }}
    .stat-val {{ font-size: 28px; font-weight: bold; color: #F1F5F9; }}
    .stat-lbl {{ font-size: 12px; color: #94A3B8; text-transform: uppercase; margin-top: 4px; }}
    table {{ width: 100%; border-collapse: collapse; margin-top: 24px; }}
    th, td {{ padding: 14px 16px; text-align: left; border-bottom: 1px solid #334155; font-size: 14px; }}
    th {{ background: #0F172A; color: #94A3B8; font-weight: 600; text-transform: uppercase; font-size: 11px; }}
    tr:hover {{ background: #334155; }}
    pre {{ background: #090D16; border: 1px solid #1E293B; border-radius: 6px; padding: 12px; font-size: 12px; color: #CBD5E1; overflow-x: auto; }}
  </style>
</head>
<body>
  <div class="container">
    <div style="display: flex; justify-content: space-between; align-items: center;">
      <h1>ReckonX Test Suite Report</h1>
      <span class="badge {'badge-pass' if failed_suites == 0 else 'badge-fail'}">
        {'ALL TESTS PASSED' if failed_suites == 0 else 'TESTS FAILED'}
      </span>
    </div>
    <div class="stats-grid">
      <div class="stat-card"><div class="stat-val">{total_suites}</div><div class="stat-lbl">Suites Run</div></div>
      <div class="stat-card"><div class="stat-val" style="color: #34D399;">{passed_suites}</div><div class="stat-lbl">Passed</div></div>
      <div class="stat-card"><div class="stat-val" style="color: #F87171;">{failed_suites}</div><div class="stat-lbl">Failed</div></div>
      <div class="stat-card"><div class="stat-val">{total_duration:.2f}s</div><div class="stat-lbl">Total Runtime</div></div>
    </div>
    <table>
      <thead>
        <tr>
          <th>Test Suite</th>
          <th>Level / Category</th>
          <th>Status</th>
          <th>Duration</th>
        </tr>
      </thead>
      <tbody>
        {"".join(f'''
        <tr>
          <td><strong>{s["name"]}</strong></td>
          <td>{s["category"]}</td>
          <td><span class="badge {'badge-pass' if s["passed"] else 'badge-fail'}" style="padding: 3px 8px; font-size: 11px;">{'PASS' if s["passed"] else 'FAIL'}</span></td>
          <td>{s["duration_s"]}s</td>
        </tr>
        ''' for s in test_suites_summary)}
      </tbody>
    </table>
  </div>
</body>
</html>
"""
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(html_content)

    print("\n" + "=" * 80)
    print("                      FULL TEST SUITE EXECUTION SUMMARY")
    print("=" * 80)
    print(f"  Total Suites Run:  {total_suites}")
    print(f"  Passed:            {passed_suites}")
    print(f"  Failed:            {failed_suites}")
    print(f"  Total Runtime:     {total_duration:.3f} seconds")
    print(f"  Status:            {'ALL TESTS PASSED' if failed_suites == 0 else 'TESTS FAILED'}")
    print("-" * 80)
    print(f"  JSON Report:       {json_path}")
    print(f"  Markdown Report:   {md_path}")
    print(f"  HTML Report:       file:///{html_path.replace(os.sep, '/')}")
    print("=" * 80)


def main():
    print("=" * 80)
    print("       RECKONX COMPREHENSIVE MULTI-TIER TEST RUNNER (Safe & Isolated)")
    print("=" * 80)

    # 1. Level 1: Python DeadReckon AI Unit Tests
    run_python_suite(
        name="Python DeadReckon AI Core (15-State EKF, HMM, INS, Seamless Controller)",
        script_path=os.path.join(ROOT_DIR, "test-runner", "python_unit_tests.py"),
        category="Unit Tests"
    )

    # 2. Level 1: Frontend & Core TypeScript Unit Tests
    run_node_ts_suite(
        name="Frontend TypeScript Core (EkfCore, InsMechanization LPF, Distance & Route Utils)",
        script_path=os.path.join(ROOT_DIR, "test-runner", "frontend_unit_tests.ts"),
        category="Unit Tests"
    )

    # 3. Level 2: Backend Fastify API Integration Tests
    run_node_ts_suite(
        name="Fastify REST API & Database (Sessions CRUD, Batch Telemetry, Route Fallbacks)",
        script_path=os.path.join(ROOT_DIR, "backend", "src", "test_api_integration.ts"),
        category="Integration Tests"
    )

    # 4. Level 3: End-to-End Mission Pipeline Test
    run_python_suite(
        name="Full Mission End-to-End Pipeline (Sensor -> INS -> AI -> 15-State EKF -> HMM Map Matching)",
        script_path=os.path.join(ROOT_DIR, "test-runner", "e2e_full_pipeline_test.py"),
        category="End-to-End Tests"
    )

    generate_reports()


if __name__ == "__main__":
    main()
