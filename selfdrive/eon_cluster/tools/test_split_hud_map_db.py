import importlib.util
import json
import sqlite3
import tempfile
import unittest
from contextlib import closing
from pathlib import Path


SCRIPT = Path(__file__).with_name("split_hud_map_db.py")
SPEC = importlib.util.spec_from_file_location("split_hud_map_db", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SplitHudMapDbTest(unittest.TestCase):
  def test_four_region_split_and_manifest(self):
    positions = {
      "south": (37.15, 127.00),
      "north": (37.80, 127.00),
      "west": (37.42, 126.90),
      "east": (37.42, 127.30),
    }
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory)
      source = root / "full.sqlite"
      output = root / "regions"
      with closing(sqlite3.connect(source)) as db:
        db.execute("CREATE TABLE tiles(z INTEGER,x INTEGER,y INTEGER,payload TEXT,"
                   "PRIMARY KEY(z,x,y))")
        db.execute("CREATE TABLE metadata(name TEXT PRIMARY KEY,value TEXT NOT NULL)")
        db.execute("INSERT INTO metadata VALUES('format','remote-hud-json-v2')")
        for region_id, (lat, lon) in positions.items():
          x, y = MODULE.tile_xy(lat, lon)
          payload = json.dumps({"b": [{"i": region_id, "p": [[lat, lon]]}],
                                "r": [], "g": [], "w": []})
          db.execute("INSERT INTO tiles VALUES(?,?,?,?)",
                     (MODULE.ZOOM, x, y, payload))
        db.commit()

      manifest_path = MODULE.split_database(str(source), str(output), "test-v1")
      manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
      self.assertEqual(manifest["format"], MODULE.FORMAT)
      self.assertEqual(manifest["version"], "test-v1")
      self.assertEqual([region["id"] for region in manifest["regions"]],
                       ["south", "north", "west", "east"])

      for region in manifest["regions"]:
        database_path = output / region["name"]
        self.assertTrue(database_path.is_file())
        self.assertEqual(region["sha256"], MODULE.sha256(database_path))
        with closing(sqlite3.connect(database_path)) as db:
          self.assertEqual(db.execute("PRAGMA integrity_check").fetchone()[0], "ok")
          metadata = dict(db.execute("SELECT name,value FROM metadata"))
          tile_count = db.execute("SELECT count(*) FROM tiles").fetchone()[0]
        self.assertEqual(metadata["region"], region["id"])
        self.assertEqual(metadata["dataset_version"], "test-v1")
        self.assertEqual(tile_count, region["tiles"])
        self.assertGreater(tile_count, 0)

  def test_selection_rectangles_do_not_overlap(self):
    selections = dict(MODULE.REGIONS)
    self.assertEqual(selections["south"]["north"], selections["west"]["south"])
    self.assertEqual(selections["west"]["north"], selections["north"]["south"])
    self.assertEqual(selections["west"]["east"], selections["east"]["west"])

  def test_incomplete_source_does_not_publish_partial_set(self):
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory)
      source = root / "partial.sqlite"
      output = root / "regions"
      lat, lon = 37.15, 127.00
      x, y = MODULE.tile_xy(lat, lon)
      with closing(sqlite3.connect(source)) as db:
        db.execute("CREATE TABLE tiles(z INTEGER,x INTEGER,y INTEGER,payload TEXT,"
                   "PRIMARY KEY(z,x,y))")
        db.execute("CREATE TABLE metadata(name TEXT PRIMARY KEY,value TEXT NOT NULL)")
        db.execute("INSERT INTO metadata VALUES('format','remote-hud-json-v2')")
        db.execute("INSERT INTO tiles VALUES(?,?,?,'{}')", (MODULE.ZOOM, x, y))
        db.commit()
      with self.assertRaisesRegex(ValueError, "region has no tiles"):
        MODULE.split_database(str(source), str(output), "partial-v1")
      self.assertEqual(list(output.glob("hud_map_gyeonggi_*.sqlite")), [])
      self.assertFalse((output / "manifest.json").exists())


if __name__ == "__main__":
  unittest.main()
