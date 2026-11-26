#!/usr/bin/env python3
"""Run the C++ solver on a Map Explorer JSON scenario."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from visualization.build import parse_grid, validate_agents

def load_case(path, maps_dir):
    obj = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(obj, dict) or obj.get("version") != 1:
        raise ValueError("expected version 1 scenario")
    if isinstance(obj.get("grid"), list):
        rows = parse_grid("\n".join(obj["grid"]))
    elif isinstance(obj.get("map"), str):
        candidates = {p.resolve() for p in (path.parent / obj["map"], maps_dir / obj["map"]) if p.is_file()}
        if len(candidates) != 1:
            raise ValueError("map reference missing or ambiguous")
        rows = parse_grid(candidates.pop().read_text(encoding="utf-8-sig"))
    else:
        raise ValueError("scenario needs grid or map")
    agents = validate_agents(rows, obj.get("agents"))
    for a, original in zip(agents, obj["agents"]):
        if a["goal"] is None:
            raise ValueError(f"agent {a['id']} has no goal")
        heading = original.get("heading", "N")
        if isinstance(heading, str) and heading in ("N", "E", "S", "W"):
            heading = "NESW".index(heading)
        if type(heading) is not int or not 0 <= heading <= 3:
            raise ValueError(f"agent {a['id']}: heading must be N/E/S/W or 0/1/2/3")
        a["heading"] = heading
    return rows, agents

def encode(rows, agents):
    lines = [f"{len(rows[0])} {len(rows)} {len(agents)}", *rows]
    for a in agents:
        lines.append(" ".join(map(str, [a["size"], *a["start"], *a["goal"], a["heading"]])))
    return "\n".join(lines) + "\n"

def executable(build_dir):
    for name in ("la_solver", "la_solver.exe", "Release/la_solver.exe", "Release/la_solver",
                 "Debug/la_solver.exe", "Debug/la_solver"):
        p = build_dir / name
        if p.is_file():
            return p.resolve()
    raise ValueError("solver executable not found; build it using CMake or pass --exe")

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("scenario", type=Path)
    p.add_argument("--exe", type=Path)
    p.add_argument("--build-dir", type=Path, default=ROOT / "build")
    p.add_argument("--maps-dir", type=Path, default=ROOT / "maps")
    p.add_argument("--solver", choices=["mc-cbs", "cbs", "joint-bfs"], default="mc-cbs")
    p.add_argument("--model", choices=["PM", "RM"], default="PM")
    p.add_argument("--policy", choices=["visit", "stay"], default="visit")
    p.add_argument("--objective", choices=["makespan", "soc"], default="makespan")
    p.add_argument("--max-horizon", type=int, default=-1)
    p.add_argument("--max-soc", type=int, default=-1)
    p.add_argument("--timeout-sec", type=float, default=300)
    p.add_argument("--report-interval-sec", type=float, default=60)
    p.add_argument("--out", type=Path, default=Path("runs/solution.json"))
    p.add_argument("--viewer", type=Path, help="optional standalone HTML replay")
    args = p.parse_args()
    try:
        rows, agents = load_case(args.scenario, args.maps_dir)
        exe = args.exe.resolve() if args.exe else executable(args.build_dir)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        progress = args.out.with_suffix(".progress.jsonl")
        cmd = [str(exe)]
        for k in ("solver", "model", "policy", "objective", "max_horizon", "max_soc",
                  "timeout_sec", "report_interval_sec"):
            cmd.extend(["--" + k.replace("_", "-"), str(getattr(args, k))])
        cmd.extend(["--progress-file", str(progress.resolve())])
        # stderr is inherited, so progress is visible live.
        run = subprocess.run(cmd, input=encode(rows, agents), text=True,
                             encoding="utf-8", stdout=subprocess.PIPE, check=True)
        data = json.loads(run.stdout)
        data["grid"], data["agents"] = rows, agents
        args.out.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({k: data[k] for k in ("status", "makespan", "soc", "lower_bound",
                                              "upper_bound", "elapsed_sec")}, ensure_ascii=False))
        print(f"Result: {args.out}; progress: {progress}")
        if args.viewer and data["paths"] is not None:
            from replay import build_replay
            build_replay(data, args.viewer)
        elif args.viewer:
            print("No solution to replay.")
    except (ValueError, OSError, subprocess.CalledProcessError) as e:
        p.exit(2, f"error: {e}\n")

if __name__ == "__main__":
    main()
