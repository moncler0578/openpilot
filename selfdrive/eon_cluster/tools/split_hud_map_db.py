#!/usr/bin/env python3
"""Split one full Gyeonggi HUD map database into four regional databases.

The selection rectangles form a non-overlapping partition. Each generated
database includes extra surrounding tiles so the Android renderer can keep its
normal z16 3x3 query while the car approaches or crosses a regional boundary.
"""

import argparse
import hashlib
import json
import math
import os
import shutil
import sqlite3
import tempfile
from contextlib import closing


ZOOM = 16
FORMAT = "remote-hud-region-manifest-v1"
PADDING_DEGREES = 0.03

# Broad bounds intentionally include the full province and a small edge margin.
PROVINCE = {"south": 36.80, "west": 126.25, "north": 38.35, "east": 128.00}
REGIONS = (
  ("south", {"south": 36.80, "west": 126.25, "north": 37.30, "east": 128.00}),
  ("north", {"south": 37.55, "west": 126.25, "north": 38.35, "east": 128.00}),
  ("west", {"south": 37.30, "west": 126.25, "north": 37.55, "east": 127.10}),
  ("east", {"south": 37.30, "west": 127.10, "north": 37.55, "east": 128.00}),
)


def tile_xy(lat, lon, zoom=ZOOM):
  scale = 1 << zoom
  x = int(math.floor((lon + 180.0) / 360.0 * scale))
  radians = math.radians(max(-85.05112878, min(85.05112878, lat)))
  y = int(math.floor((1.0 - math.log(math.tan(radians)
                                      + 1.0 / math.cos(radians)) / math.pi)
                      * 0.5 * scale))
  return max(0, min(scale - 1, x)), max(0, min(scale - 1, y))


def tile_range(bounds, padding=PADDING_DEGREES):
  south = max(-85.0, bounds["south"] - padding)
  north = min(85.0, bounds["north"] + padding)
  west = max(-180.0, bounds["west"] - padding)
  east = min(180.0, bounds["east"] + padding)
  x0, y_south = tile_xy(south, west)
  x1, y_north = tile_xy(north, east)
  return min(x0, x1), max(x0, x1), min(y_north, y_south), max(y_north, y_south)


def sha256(path):
  digest = hashlib.sha256()
  with open(path, "rb") as stream:
    for block in iter(lambda: stream.read(1024 * 1024), b""):
      digest.update(block)
  return digest.hexdigest()


def validate_source(path):
  with closing(sqlite3.connect(
      "file:%s?mode=ro" % os.path.abspath(path), uri=True)) as db:
    result = db.execute("PRAGMA integrity_check").fetchone()[0]
    if result != "ok":
      raise ValueError("source integrity check failed: %s" % result)
    count = db.execute("SELECT count(*) FROM tiles WHERE z=?", (ZOOM,)).fetchone()[0]
    if count <= 0:
      raise ValueError("source database has no z%d tiles" % ZOOM)


def write_region(source, output, region_id, selection, dataset_version):
  directory = os.path.dirname(os.path.abspath(output))
  os.makedirs(directory, exist_ok=True)
  descriptor, temporary = tempfile.mkstemp(prefix=".hud-region-", suffix=".sqlite",
                                            dir=directory)
  os.close(descriptor)
  os.remove(temporary)
  x0, x1, y0, y1 = tile_range(selection)
  try:
    with closing(sqlite3.connect(temporary)) as db:
      db.execute("PRAGMA journal_mode=OFF")
      db.execute("PRAGMA synchronous=OFF")
      db.execute("CREATE TABLE tiles(z INTEGER,x INTEGER,y INTEGER,payload TEXT,"
                 "PRIMARY KEY(z,x,y))")
      db.execute("CREATE TABLE metadata(name TEXT PRIMARY KEY,value TEXT NOT NULL)")
      db.execute("ATTACH DATABASE ? AS source", (os.path.abspath(source),))
      db.execute("INSERT INTO tiles SELECT z,x,y,payload FROM source.tiles "
                 "WHERE z=? AND x BETWEEN ? AND ? AND y BETWEEN ? AND ?",
                 (ZOOM, x0, x1, y0, y1))
      db.execute("INSERT OR IGNORE INTO metadata SELECT name,value FROM source.metadata")
      tile_count = db.execute("SELECT count(*) FROM tiles").fetchone()[0]
      if tile_count <= 0:
        raise ValueError("%s region has no tiles" % region_id)
      values = {
        "format": "remote-hud-json-v2",
        "dataset_version": dataset_version,
        "region": region_id,
        "tile_count": str(tile_count),
        "selection_bounds": json.dumps(selection, separators=(",", ":")),
        "tile_range": "%d,%d,%d,%d" % (x0, x1, y0, y1),
      }
      db.executemany("INSERT OR REPLACE INTO metadata VALUES(?,?)", values.items())
      db.commit()
      db.execute("DETACH DATABASE source")
      db.execute("VACUUM")
      integrity = db.execute("PRAGMA integrity_check").fetchone()[0]
      if integrity != "ok":
        raise ValueError("%s integrity check failed: %s" % (region_id, integrity))
    os.replace(temporary, output)
  finally:
    if os.path.exists(temporary):
      os.remove(temporary)

  return {
    "id": region_id,
    "name": "hud_map_gyeonggi_%s.sqlite" % region_id,
    "selection": selection,
    "sha256": sha256(output),
    "bytes": os.path.getsize(output),
    "tiles": tile_count,
  }


def split_database(source, output_dir, dataset_version):
  validate_source(source)
  os.makedirs(output_dir, exist_ok=True)
  staging = tempfile.mkdtemp(prefix=".hud-regions-", dir=output_dir)
  try:
    manifest = {
      "format": FORMAT,
      "version": dataset_version,
      "zoom": ZOOM,
      "province": PROVINCE,
      "regions": [],
    }
    for region_id, selection in REGIONS:
      output = os.path.join(staging, "hud_map_gyeonggi_%s.sqlite" % region_id)
      manifest["regions"].append(
        write_region(source, output, region_id, selection, dataset_version))
    staged_manifest = os.path.join(staging, "manifest.json")
    with open(staged_manifest, "w", encoding="utf-8") as stream:
      json.dump(manifest, stream, ensure_ascii=False, indent=2, sort_keys=True)
      stream.write("\n")

    for region in manifest["regions"]:
      os.replace(os.path.join(staging, region["name"]),
                 os.path.join(output_dir, region["name"]))
    manifest_path = os.path.join(output_dir, "manifest.json")
    os.replace(staged_manifest, manifest_path)
    return manifest_path
  finally:
    shutil.rmtree(staging, ignore_errors=True)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--input", required=True, help="full Gyeonggi hud_map.sqlite")
  parser.add_argument("--output-dir", required=True)
  parser.add_argument("--version", default="gyeonggi-v1")
  args = parser.parse_args()
  path = split_database(args.input, args.output_dir, args.version)
  print(path)


if __name__ == "__main__":
  main()
