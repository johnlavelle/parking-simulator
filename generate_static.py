#!/usr/bin/env python3
"""Generate pre-computed simulation data and a static site for GitHub Pages."""

import json
import sys
from pathlib import Path

from app.geometry import (
    get_building_polygon_wgs84,
    get_parking_area_wgs84,
    get_right_of_way_polygon_wgs84,
    get_row_min_width,
    polygon_to_geojson,
)
from app.models import CAR_DIMENSIONS, CarType
from app.optimizer import optimize_placement
from app.pathfinder import compute_drive_out_paths

ROW_WIDTH_MARGIN = 0.15
MAX_CARS = 4

def main():
    out_dir = Path("docs")
    out_dir.mkdir(exist_ok=True)

    # --- Polygons ---
    polygons = {
        "building": polygon_to_geojson(get_building_polygon_wgs84()),
        "parking_area": polygon_to_geojson(get_parking_area_wgs84()),
        "right_of_way": polygon_to_geojson(get_right_of_way_polygon_wgs84()),
    }

    # --- Config ---
    min_width = get_row_min_width()
    max_car_width = min_width - ROW_WIDTH_MARGIN
    car_types = []
    for ct in CarType:
        length, width = CAR_DIMENSIONS[ct]
        if width <= max_car_width:
            car_types.append({
                "value": ct.value,
                "label": f"{ct.value} ({length:.2f} \u00d7 {width:.2f}m)",
                "length": length,
                "width": width,
            })

    config = {
        "max_cars": MAX_CARS,
        "row_min_width": round(min_width, 2),
        "max_car_width": round(max_car_width, 2),
        "car_types": car_types,
    }

    # --- Simulations ---
    simulations = {}
    allowed_types = [ct["value"] for ct in car_types]

    for ct in CarType:
        if ct.value not in allowed_types:
            print(f"  Skipping {ct.value} (too wide for ROW)")
            continue
        car_length, car_width = CAR_DIMENSIONS[ct]
        for n in range(1, MAX_CARS + 1):
            key = f"{ct.value}_{n}"
            print(f"  Simulating {key}...", end=" ", flush=True)
            placements, warnings = optimize_placement(n, ct)
            paths = compute_drive_out_paths(placements)

            simulations[key] = {
                "placements": [p.model_dump() for p in placements],
                "paths": [p.model_dump() for p in paths],
                "warnings": warnings,
                "car_length": car_length,
                "car_width": car_width,
            }
            placed = len(placements)
            path_count = len(paths)
            print(f"{placed} placed, {path_count} paths" +
                  (f", warnings: {warnings}" if warnings else ""))

    # --- Write data.js ---
    data = {
        "config": config,
        "polygons": polygons,
        "simulations": simulations,
    }

    data_js = out_dir / "data.js"
    data_js.write_text(
        "// Auto-generated simulation data — do not edit\n"
        f"const SIM_DATA = {json.dumps(data, separators=(',', ':'))};\n"
    )
    print(f"\nWrote {data_js} ({data_js.stat().st_size / 1024:.1f} KB)")


if __name__ == "__main__":
    main()
