import json
from pathlib import Path
import tempfile
import unittest
from visualization.build import build, collect, parse_grid

class BuildTests(unittest.TestCase):
    def test_parsing(self):
        self.assertEqual(parse_grid("type octile\nheight 1\nwidth 4\nmap\n.G@T"), ["..##"])
        with self.assertRaises(ValueError):
            parse_grid("..\n.")

    def test_catalog_and_build(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / "maps"
            folder.mkdir()
            (folder / "a.grid").write_text("...\n...\n...\n", encoding="utf-8")
            (folder / "case.json").write_text(json.dumps({
                "version": 1, "map": "a.grid",
                "agents": [{"id": 0, "size": 3, "start": [1, 1], "goal": None}]
            }), encoding="utf-8")
            entries = collect(folder)
            self.assertEqual(len(entries), 2)
            self.assertEqual(entries[1]["agents"][0]["size"], 3)
            out = Path(tmp) / "explorer.html"
            build(folder, out)
            content = out.read_text(encoding="utf-8")
            self.assertIn('"name":"case.json"', content)
            self.assertNotIn("/*__CATALOG__*/", content)
            self.assertNotIn("/*__APP__*/", content)
            self.assertIn("const Geo =", content)

    def test_bad_scenario(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder / "bad.json").write_text(json.dumps({
                "version": 1, "grid": ["."],
                "agents": [{"id": 0, "size": 3, "start": [0, 0], "goal": None}]
            }), encoding="utf-8")
            with self.assertRaises(ValueError):
                collect(folder)

if __name__ == "__main__":
    unittest.main()
