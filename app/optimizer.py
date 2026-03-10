"""Car placement optimizer using grid search over the parking area."""

import math
from shapely.geometry import Polygon, Point
from app.geometry import (
    get_parking_area_wgs84,
    get_building_polygon_wgs84,
    get_right_of_way_polygon_wgs84,
    polygon_to_local,
    local_to_wgs84,
)
from app.models import CarType, CAR_DIMENSIONS, CarPlacement

BUILDING_CLEARANCE = 0.3  # meters from building for maneuverability


GRID_STEP = 0.2  # meters
MIN_CLEARANCE = 0.30  # meters between cars


def _create_car_rect(
    cx: float, cy: float, length: float, width: float, angle_deg: float
) -> Polygon:
    """Create a rotated rectangle centered at (cx, cy) in local meters.
    Length is along the angle direction, width is perpendicular.
    """
    angle_rad = math.radians(angle_deg)
    hl, hw = length / 2, width / 2
    corners = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    rotated = []
    for dx, dy in corners:
        rx = cx + dx * cos_a - dy * sin_a
        ry = cy + dx * sin_a + dy * cos_a
        rotated.append((rx, ry))
    return Polygon(rotated)


def _find_dominant_angle(poly: Polygon) -> float:
    """Find the angle of the longest edge of the polygon's exterior."""
    coords = list(poly.exterior.coords)
    best_len = 0
    best_angle = 0
    for i in range(len(coords) - 1):
        dx = coords[i + 1][0] - coords[i][0]
        dy = coords[i + 1][1] - coords[i][1]
        edge_len = math.hypot(dx, dy)
        if edge_len > best_len:
            best_len = edge_len
            best_angle = math.degrees(math.atan2(dy, dx))
    return best_angle


def _find_exit_point(parking_m: Polygon) -> Point:
    """Find the ROW exit point at Shore Road (farthest from parking centroid)."""
    row_m = polygon_to_local(get_right_of_way_polygon_wgs84())
    parking_centroid = parking_m.centroid
    best_dist = 0
    best_pt = row_m.centroid
    for coord in row_m.exterior.coords:
        p = Point(coord)
        d = p.distance(parking_centroid)
        if d > best_dist:
            best_dist = d
            best_pt = p
    return best_pt



def _compute_heading(cx: float, cy: float, angle: float, exit_pt: Point) -> float:
    """Compute heading so the FRONT faces toward Shore Road (exit).

    Cars drive forward out through the corridor to Shore Road.
    The heading is chosen from the two possible orientations (angle, angle+180)
    to best face the exit direction.
    """
    exit_dir = math.degrees(math.atan2(exit_pt.y - cy, exit_pt.x - cx))
    diff = (exit_dir - angle + 180) % 360 - 180
    if abs(diff) <= 90:
        return angle
    else:
        return angle + 180


def optimize_placement(
    num_cars: int, car_type: CarType
) -> tuple[list[CarPlacement], list[str]]:
    """Place num_cars of car_type in the parking area.

    Returns (placements, warnings).
    """
    parking_wgs84 = get_parking_area_wgs84()
    parking_m = polygon_to_local(parking_wgs84)
    car_length, car_width = CAR_DIMENSIONS[car_type]

    exit_pt = _find_exit_point(parking_m)
    building_m = polygon_to_local(get_building_polygon_wgs84())
    dominant_angle = _find_dominant_angle(parking_m)
    angles = [dominant_angle, dominant_angle + 90]

    # Generate candidate placements
    minx, miny, maxx, maxy = parking_m.bounds
    candidates: list[tuple[float, float, float, float, Polygon, float]] = []

    y = miny
    while y <= maxy:
        x = minx
        while x <= maxx:
            pt = Point(x, y)
            if parking_m.contains(pt):
                exit_dir = math.atan2(exit_pt.y - y, exit_pt.x - x)
                for angle in angles:
                    rect = _create_car_rect(x, y, car_length, car_width, angle)
                    if parking_m.contains(rect) and rect.distance(building_m) >= BUILDING_CLEARANCE:
                        dist = pt.distance(exit_pt)
                        heading = _compute_heading(x, y, angle, exit_pt)
                        # Measure how well the FRONT faces the exit
                        front_dir = math.radians(heading)
                        heading_diff = abs(
                            (exit_dir - front_dir + math.pi)
                            % (2 * math.pi)
                            - math.pi
                        )
                        candidates.append((x, y, angle, dist, rect, heading_diff))
            x += GRID_STEP
        y += GRID_STEP

    # Sort by weighted score: prefer far from exit AND front aligned with exit
    HEADING_PENALTY = 3.0  # meters per radian

    def _greedy_place(cands, n):
        cands_sorted = sorted(cands, key=lambda c: -(c[3] - HEADING_PENALTY * c[5]))
        result = []
        for cx, cy, angle, dist, rect, heading_diff in cands_sorted:
            if len(result) >= n:
                break
            overlap = False
            for _, _, _, _, placed_rect, _ in result:
                if rect.distance(placed_rect) < MIN_CLEARANCE:
                    overlap = True
                    break
            if not overlap:
                result.append((cx, cy, angle, dist, rect, heading_diff))
        return result

    # Try each angle independently and pick the one that fits more cars
    best_placed: list[tuple[float, float, float, float, Polygon, float]] = []
    for angle in angles:
        angle_candidates = [c for c in candidates if c[2] == angle]
        result = _greedy_place(angle_candidates, num_cars)
        if len(result) > len(best_placed):
            best_placed = result

    # Also try mixed angles (original behavior) in case it fits more
    mixed = _greedy_place(candidates, num_cars)
    if len(mixed) > len(best_placed):
        best_placed = mixed

    placed = best_placed

    # Convert to response models
    warnings: list[str] = []
    if len(placed) < num_cars:
        warnings.append(
            f"Could only fit {len(placed)} of {num_cars} requested "
            f"{car_type.value} cars"
        )

    placements: list[CarPlacement] = []
    for i, (cx, cy, angle, _, rect, _) in enumerate(placed):
        heading = _compute_heading(cx, cy, angle, exit_pt)
        center_wgs84 = local_to_wgs84(cx, cy)
        corners_wgs84 = [local_to_wgs84(x, y) for x, y in rect.exterior.coords[:-1]]
        placements.append(
            CarPlacement(
                car_index=i,
                center=list(center_wgs84),
                corners=[list(c) for c in corners_wgs84],
                angle=angle,
                heading=heading,
                car_type=car_type,
            )
        )

    return placements, warnings
