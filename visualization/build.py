"""Build a standalone offline Map Explorer using only Python stdlib."""
import argparse
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

def parse_grid(text):
    rows = [s.strip() for s in text.lstrip("\ufeff").strip().splitlines()]
    if rows and rows[0].lower().startswith("type "):
        if len(rows) < 5 or rows[3].lower() != "map":
            raise ValueError("invalid MovingAI header")
        h, w = rows[1].split(), rows[2].split()
        if h[0].lower() != "height" or w[0].lower() != "width":
            raise ValueError("invalid dimensions")
        rows = rows[4:]
        if len(rows) != int(h[1]) or any(len(s) != int(w[1]) for s in rows):
            raise ValueError("dimensions do not match header")
        trans = {c: "." for c in ".GS"} | {c: "#" for c in "@OTW"}
        rows = ["".join(trans[c] for c in s) for s in rows]
    if not rows or not rows[0] or any(len(s) != len(rows[0]) or set(s) - {".", "#"} for s in rows):
        raise ValueError("expected rectangular . / # grid")
    if len(rows) * len(rows[0]) > 1_000_000:
        raise ValueError("map exceeds 1,000,000 cells")
    return rows

def validate_agents(rows, agents):
    if not isinstance(agents, list):
        raise ValueError("agents must be a list")
    w, h = len(rows[0]), len(rows)
    ids = set()
    def fits(p, size):
        if not isinstance(p, list) or len(p) != 2 or any(type(v) is not int for v in p):
            return False
        x, y = p
        k = size // 2
        return (k <= x < w-k and k <= y < h-k and
                all("#" not in rows[j][x-k:x+k+1] for j in range(y-k, y+k+1)))
    clean = []
    for a in agents:
        if not isinstance(a, dict):
            raise ValueError("agent must be an object")
        i, s, p, q = (a.get(k) for k in ("id", "size", "start", "goal"))
        if (type(i) is not int or not 0 <= i <= 2**53-1 or i in ids or
            type(s) is not int or s < 1 or s % 2 == 0 or not fits(p, s) or
            "goal" not in a or (q is not None and not fits(q, s))):
            raise ValueError(f"invalid agent: {i}")
        for b in clean:
            if all(2*abs(p[j]-b["start"][j]) < s+b["size"] for j in range(2)):
                raise ValueError("overlapping initial footprints")
        ids.add(i)
        clean.append({"id": i, "size": s, "start": p, "goal": q})
    return clean

def collect(folder):
    if not folder.is_dir():
        raise ValueError(f"not a directory: {folder}")
    maps = []
    for path in sorted(folder.rglob("*")):
        if path.is_file() and path.suffix.lower() in {".grid", ".map"}:
            try:
                rows = parse_grid(path.read_text(encoding="utf-8-sig"))
            except (ValueError, KeyError, IndexError) as exc:
                raise ValueError(f"{path}: {exc}") from exc
            maps.append({"name": path.relative_to(folder).as_posix(), "grid": rows, "agents": []})
    by_name = {m["name"]: m["grid"] for m in maps}
    for path in sorted(folder.rglob("*")):
        if not path.is_file() or path.suffix.lower() != ".json":
            continue
        try:
            obj = json.loads(path.read_text(encoding="utf-8-sig"))
            if not isinstance(obj, dict) or type(obj.get("version")) is not int or obj["version"] != 1:
                raise ValueError("expected scenario version 1")
            if isinstance(obj.get("grid"), list):
                rows = parse_grid("\n".join(obj["grid"]))
            elif isinstance(obj.get("map"), str):
                relative = (path.parent.relative_to(folder) / obj["map"]).as_posix()
                keys = {k for k in (relative, obj["map"]) if k in by_name}
                if len(keys) != 1:
                    raise ValueError("map reference missing or ambiguous")
                rows = by_name[keys.pop()]
            else:
                raise ValueError("expected grid or map")
            agents = validate_agents(rows, obj.get("agents"))
            maps.append({"name": path.relative_to(folder).as_posix(), "grid": rows, "agents": agents})
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            raise ValueError(f"{path}: {exc}") from exc
    return sorted(maps, key=lambda m: m["name"])

def build(folder, output):
    maps = collect(folder)
    catalog = json.dumps(maps, ensure_ascii=True, separators=(",", ":")).replace("<", "\\u003c")
    template = (HERE / "template.html").read_text(encoding="utf-8")
    for marker, value in {
        "CSS": (HERE / "style.css").read_text(encoding="utf-8"),
        "CORE": (HERE / "geometry.js").read_text(encoding="utf-8"),
        "APP": (HERE / "app.js").read_text(encoding="utf-8"),
        "CATALOG": catalog,
    }.items():
        token = "/*__" + marker + "__*/"
        if template.count(token) != 1:
            raise ValueError(f"missing or duplicate marker: {marker}")
        template = template.replace(token, value)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(template, encoding="utf-8", newline="\n")
    print(f"{len(maps)} maps -> {output} ({output.stat().st_size:,} bytes)")

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--maps-dir", type=Path, default=ROOT / "maps")
    p.add_argument("--out", type=Path, default=ROOT / "dist" / "explorer.html")
    a = p.parse_args()
    build(a.maps_dir, a.out)
