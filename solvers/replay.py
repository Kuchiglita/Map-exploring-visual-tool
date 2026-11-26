"""Generate an offline trajectory replay from a solver result."""
import argparse
import json
from pathlib import Path

def build_replay(data, out):
    if data.get("paths") is None:
        raise ValueError("result has no solution")
    template = Path(__file__).with_name("replay.html").read_text(encoding="utf-8")
    payload = json.dumps(data, ensure_ascii=True, separators=(",", ":")).replace("<", "\\u003c")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(template.replace("/*__DATA__*/", payload), encoding="utf-8")
    print(f"Replay: {out}")

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("result", type=Path)
    p.add_argument("--out", type=Path, default=Path("runs/replay.html"))
    a = p.parse_args()
    build_replay(json.loads(a.result.read_text(encoding="utf-8")), a.out)
