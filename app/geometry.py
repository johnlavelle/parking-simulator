"""Polygon definitions and coordinate transforms for 29 Shore Road parking simulator.

All source polygons are WGS84 coordinates extracted from shapefiles.
Parking area is computed as building_outline_inc_yard MINUS building_outline.
"""

import math
from shapely.geometry import Polygon, MultiPolygon, LineString, Point
from shapely.validation import make_valid

# Reference latitude for equirectangular projection
REF_LAT = 54.6426
# Meters per degree at this latitude
DEG_LON_TO_M = math.cos(math.radians(REF_LAT)) * 111_320  # ~64,380 m
DEG_LAT_TO_M = 111_320

# Reference point for local coordinate system (centroid of building_outline_inc_yard)
REF_LON = -5.83543643
REF_LAT_PT = 54.64258140

# --- Raw polygon coordinates (lon, lat) from shapefiles ---

BUILDING_OUTLINE_INC_YARD_WGS84 = [
    (-5.83548211, 54.64249628),
    (-5.83555022, 54.64252144),
    (-5.83539076, 54.64266651),
    (-5.83532274, 54.64264138),
    (-5.83548211, 54.64249628),
]

BUILDING_OUTLINE_WGS84 = [
    (-5.83548135, 54.64249627),
    (-5.83555022, 54.64252144),
    (-5.83547941, 54.64258590),
    (-5.83544627, 54.64261610),
    (-5.83539615, 54.64259549),
    (-5.83542790, 54.64256616),
    (-5.83541151, 54.64256020),
    (-5.83548135, 54.64249627),
]

# Raw ROW polygon has a duplicate vertex at index 2 (same as index 0).
# We store it raw and clean it with Shapely.
RIGHT_OF_WAY_RAW_WGS84 = [
    (-5.835322743963921, 54.64264137836603),
    (-5.835283070264921, 54.64266381852416),
    (-5.835322743963921, 54.64264137836603),  # duplicate of vertex 0
    (-5.835390762855520, 54.64266650793813),
    (-5.835550216885508, 54.64252144381967),
    (-5.835576877775627, 54.64253204004057),
    (-5.835388459431987, 54.64270312307733),
    (-5.835283020505406, 54.64266383258284),
    (-5.835322743963921, 54.64264137836603),
]


def wgs84_to_local(lon: float, lat: float) -> tuple[float, float]:
    """Convert WGS84 (lon, lat) to local meters relative to reference point."""
    x = (lon - REF_LON) * DEG_LON_TO_M
    y = (lat - REF_LAT_PT) * DEG_LAT_TO_M
    return (x, y)


def local_to_wgs84(x: float, y: float) -> tuple[float, float]:
    """Convert local meters to WGS84 (lon, lat)."""
    lon = x / DEG_LON_TO_M + REF_LON
    lat = y / DEG_LAT_TO_M + REF_LAT_PT
    return (lon, lat)


def _to_local_ring(coords: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Convert a WGS84 coordinate ring to local meters."""
    return [wgs84_to_local(lon, lat) for lon, lat in coords]


def _to_wgs84_ring(coords: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Convert a local-meters coordinate ring to WGS84."""
    return [local_to_wgs84(x, y) for x, y in coords]


def _clean_polygon(poly: Polygon) -> Polygon:
    """Fix an invalid polygon, returning the largest polygon if result is multi."""
    if poly.is_valid:
        return poly
    fixed = make_valid(poly)
    if isinstance(fixed, Polygon):
        return fixed
    if isinstance(fixed, MultiPolygon):
        return max(fixed.geoms, key=lambda g: g.area)
    # GeometryCollection - extract largest polygon
    polys = [g for g in fixed.geoms if isinstance(g, Polygon)]
    if polys:
        return max(polys, key=lambda g: g.area)
    return poly  # fallback


def get_building_polygon_wgs84() -> Polygon:
    return Polygon(BUILDING_OUTLINE_WGS84)


def get_building_inc_yard_polygon_wgs84() -> Polygon:
    return Polygon(BUILDING_OUTLINE_INC_YARD_WGS84)


def get_right_of_way_polygon_wgs84() -> Polygon:
    raw = Polygon(RIGHT_OF_WAY_RAW_WGS84)
    return _clean_polygon(raw)


def get_parking_area_wgs84() -> Polygon:
    """Parking area = building_outline_inc_yard MINUS building_outline."""
    yard = get_building_inc_yard_polygon_wgs84()
    building = get_building_polygon_wgs84()
    diff = yard.difference(building)
    if isinstance(diff, MultiPolygon):
        return max(diff.geoms, key=lambda g: g.area)
    return diff


def polygon_to_local(poly: Polygon) -> Polygon:
    """Convert a WGS84 polygon to local meters."""
    ext = _to_local_ring(poly.exterior.coords)
    holes = [_to_local_ring(h.coords) for h in poly.interiors]
    return Polygon(ext, holes)


def polygon_to_wgs84(poly: Polygon) -> Polygon:
    """Convert a local-meters polygon to WGS84."""
    ext = _to_wgs84_ring(poly.exterior.coords)
    holes = [_to_wgs84_ring(h.coords) for h in poly.interiors]
    return Polygon(ext, holes)


def get_row_min_width() -> float:
    """Compute the minimum width of the ROW corridor in meters.

    Measures the distance between the two long edges of the ROW polygon
    at many sample points and returns the minimum.
    """
    row_m = polygon_to_local(get_right_of_way_polygon_wgs84())
    coords = list(row_m.exterior.coords[:-1])

    # Find the two longest edges (corridor sides)
    edges = []
    for i in range(len(coords)):
        j = (i + 1) % len(coords)
        dx = coords[j][0] - coords[i][0]
        dy = coords[j][1] - coords[i][1]
        edges.append((i, j, math.hypot(dx, dy)))
    edges.sort(key=lambda e: -e[2])

    edge1 = LineString([coords[edges[0][0]], coords[edges[0][1]]])
    edge2 = LineString([coords[edges[1][0]], coords[edges[1][1]]])

    min_width = float("inf")
    for t in range(101):
        tn = t / 100
        p1 = edge1.interpolate(tn, normalized=True)
        p2 = edge2.interpolate(tn, normalized=True)
        w = p1.distance(p2)
        if w < min_width:
            min_width = w
    return min_width


def polygon_to_geojson(poly: Polygon) -> dict:
    """Convert a Shapely polygon to GeoJSON-like dict with [lon, lat] coords."""
    coords = [list(poly.exterior.coords)]
    for hole in poly.interiors:
        coords.append(list(hole.coords))
    return {
        "type": "Polygon",
        "coordinates": [[[c[0], c[1]] for c in ring] for ring in coords],
    }
