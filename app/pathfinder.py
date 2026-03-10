"""Drive-out path computation with bicycle-model simulation and boundary enforcement."""

import math
from shapely.geometry import Point, LineString, Polygon
from app.geometry import (
    get_parking_area_wgs84,
    get_building_polygon_wgs84,
    get_right_of_way_polygon_wgs84,
    polygon_to_local,
    local_to_wgs84,
    wgs84_to_local,
)
from app.models import CarPlacement, DriveOutPath, CAR_DIMENSIONS

NUM_CENTERLINE_POINTS = 30
DENSIFY_SPACING = 0.4
OBSTACLE_BUFFER = 0.3

# Bicycle model constants — realistic turning circles
SPEED = 3.0  # m/s
DT = 0.05  # seconds per simulation step
MAX_STEER_DEG = 35  # realistic max front-wheel steering angle
REAR_AXLE_RATIO = 0.25  # rear axle at 25% from back
WHEELBASE_FACTOR = 0.6  # realistic wheelbase ratio
LOOK_AHEAD_FACTOR = 1.5
ARRIVAL_DIST = 3.0  # m
MAX_SIM_STEPS = 12000
MIN_INSIDE_RATIO = 0.55  # allow corner clipping (~45%) for tight turns near building
MAX_BUILDING_OVERLAP = 0.20  # allow up to 20% of car to clip building during maneuvers
BUMP_STEPS = 30  # forward bump steps when stuck in reverse
BUMP_SPEED_FACTOR = 0.4
FRAME_SKIP = 2  # send every Nth simulation step to frontend


def _compute_safe_centerline(
    row_m: Polygon, parking_m: Polygon, car_width: float
) -> list[tuple[float, float]]:
    """Compute centerline through the ROW, inset by half car width so
    the car body stays within the ROW boundary.
    """
    safe_margin = car_width / 2 + 0.05
    safe_row = row_m.buffer(-safe_margin)
    if safe_row.is_empty or safe_row.area < 0.5:
        safe_row = row_m  # fallback if ROW too narrow

    return _compute_raw_centerline(safe_row, parking_m)


def _compute_raw_centerline(
    row_m: Polygon, parking_m: Polygon
) -> list[tuple[float, float]]:
    """Boundary-splitting centerline through a polygon."""
    parking_centroid = parking_m.centroid
    row_boundary = row_m.exterior
    total_length = row_boundary.length

    entry_dist = row_boundary.project(parking_centroid)

    best_exit_dist = 0
    best_exit_pos = 0
    for i in range(200):
        pos = total_length * i / 200
        d = row_boundary.interpolate(pos).distance(parking_centroid)
        if d > best_exit_dist:
            best_exit_dist = d
            best_exit_pos = pos

    entry_pos, exit_pos = entry_dist, best_exit_pos
    if entry_pos > exit_pos:
        entry_pos, exit_pos = exit_pos, entry_pos

    half1 = exit_pos - entry_pos
    half2 = total_length - half1

    centerline: list[tuple[float, float]] = []
    for i in range(NUM_CENTERLINE_POINTS + 1):
        t = i / NUM_CENTERLINE_POINTS
        p1 = row_boundary.interpolate(entry_pos + t * half1)
        p2 = row_boundary.interpolate((entry_pos - t * half2) % total_length)
        centerline.append(((p1.x + p2.x) / 2, (p1.y + p2.y) / 2))

    # Keep only points inside or very near the polygon
    inside = [
        pt for pt in centerline
        if row_m.contains(Point(pt)) or row_m.boundary.distance(Point(pt)) < 0.5
    ] or centerline

    # Filter to monotonically decreasing distance to exit (remove backtracking)
    if len(inside) < 2:
        return inside
    exit_pt = inside[-1]
    filtered = [inside[0]]
    best_dist = math.hypot(inside[0][0] - exit_pt[0], inside[0][1] - exit_pt[1])
    for pt in inside[1:]:
        d = math.hypot(pt[0] - exit_pt[0], pt[1] - exit_pt[1])
        if d < best_dist - 0.1:
            best_dist = d
            filtered.append(pt)
    if filtered[-1] != inside[-1]:
        filtered.append(inside[-1])
    return filtered


def _smooth_path(points: list[tuple[float, float]], iterations: int = 3) -> list[tuple[float, float]]:
    """Chaikin's corner-cutting for smooth curves."""
    for _ in range(iterations):
        if len(points) < 3:
            break
        new = [points[0]]
        for i in range(len(points) - 1):
            ax, ay = points[i]
            bx, by = points[i + 1]
            new.append((0.75 * ax + 0.25 * bx, 0.75 * ay + 0.25 * by))
            new.append((0.25 * ax + 0.75 * bx, 0.25 * ay + 0.75 * by))
        new.append(points[-1])
        points = new
    return points


def _cubic_bezier(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    n_points: int = 20,
) -> list[tuple[float, float]]:
    """Cubic Bezier curve from p0 to p3 with control points p1, p2."""
    pts = []
    for i in range(n_points + 1):
        t = i / n_points
        mt = 1 - t
        x = mt**3 * p0[0] + 3 * mt**2 * t * p1[0] + 3 * mt * t**2 * p2[0] + t**3 * p3[0]
        y = mt**3 * p0[1] + 3 * mt**2 * t * p1[1] + 3 * mt * t**2 * p2[1] + t**3 * p3[1]
        pts.append((x, y))
    return pts


def _densify(points: list[tuple[float, float]], spacing: float) -> list[tuple[float, float]]:
    if len(points) < 2:
        return points
    result = [points[0]]
    for i in range(1, len(points)):
        px, py = result[-1]
        nx, ny = points[i]
        d = math.hypot(nx - px, ny - py)
        if d > spacing:
            n = max(1, int(math.ceil(d / spacing)))
            for j in range(1, n):
                t = j / n
                result.append((px + t * (nx - px), py + t * (ny - py)))
        result.append(points[i])
    return result


def _reroute_around_building(
    path: list[tuple[float, float]],
    building: Polygon,
    maneuvering: Polygon,
) -> list[tuple[float, float]]:
    """Reroute a path so no segment crosses through the building.

    When consecutive segments intersect the building, replace the offending
    section with points along the building boundary (buffered outward),
    choosing the shorter way around.
    """
    if len(path) < 2 or not building.is_valid:
        return path

    line = LineString(path)
    if not line.intersects(building):
        return path

    # Buffer the building slightly so the detour stays clear of it
    detour_boundary = building.buffer(0.5).exterior

    result: list[tuple[float, float]] = []
    i = 0
    while i < len(path):
        pt = path[i]
        if building.contains(Point(pt)):
            # Find the run of points inside the building
            j = i
            while j < len(path) and building.contains(Point(path[j])):
                j += 1
            # entry = last good point, exit = first good point after
            entry = path[i - 1] if i > 0 else path[i]
            exit_pt = path[j] if j < len(path) else path[-1]

            # Project entry and exit onto the detour boundary
            entry_d = detour_boundary.project(Point(entry))
            exit_d = detour_boundary.project(Point(exit_pt))
            total_len = detour_boundary.length

            # Two ways around: forward and backward along the boundary ring
            if entry_d <= exit_d:
                fwd_len = exit_d - entry_d
                bwd_len = total_len - fwd_len
            else:
                fwd_len = total_len - entry_d + exit_d
                bwd_len = entry_d - exit_d

            # Pick shorter direction
            if fwd_len <= bwd_len:
                # Go forward (increasing distance)
                dist = entry_d
                step = DENSIFY_SPACING
                while True:
                    dist += step
                    if dist >= total_len:
                        dist -= total_len
                    # Check if we've passed exit_d (handle wrap)
                    if fwd_len <= bwd_len:
                        passed = (dist - entry_d) % total_len >= fwd_len - step / 2
                    else:
                        passed = False
                    if passed:
                        break
                    p = detour_boundary.interpolate(dist)
                    if maneuvering.contains(p) or maneuvering.boundary.distance(p) < 0.3:
                        result.append((p.x, p.y))
            else:
                # Go backward (decreasing distance)
                dist = entry_d
                step = DENSIFY_SPACING
                while True:
                    dist -= step
                    if dist < 0:
                        dist += total_len
                    passed = (entry_d - dist) % total_len >= bwd_len - step / 2
                    if passed:
                        break
                    p = detour_boundary.interpolate(dist)
                    if maneuvering.contains(p) or maneuvering.boundary.distance(p) < 0.3:
                        result.append((p.x, p.y))

            i = j  # skip past the building section
        else:
            result.append(pt)
            i += 1

    return result if len(result) >= 2 else path


def _build_car_rect(placement: CarPlacement) -> Polygon:
    corners = [wgs84_to_local(c[0], c[1]) for c in placement.corners]
    return Polygon(corners)


def _car_rect_from_rear(
    rx: float, ry: float, heading: float,
    car_length: float, car_width: float,
) -> Polygon:
    """Build car rectangle from rear axle position and heading."""
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    rear_overhang = car_length * REAR_AXLE_RATIO
    front_dist = car_length - rear_overhang
    hw = car_width / 2
    local = [
        (-rear_overhang, hw), (front_dist, hw),
        (front_dist, -hw), (-rear_overhang, -hw),
    ]
    return Polygon([
        (rx + lx * cos_h - ly * sin_h, ry + lx * sin_h + ly * cos_h)
        for lx, ly in local
    ])


def _rear_to_center(rx, ry, heading, car_length):
    offset = car_length * (0.5 - REAR_AXLE_RATIO)
    return (rx + offset * math.cos(heading), ry + offset * math.sin(heading))


def _center_to_rear(cx, cy, heading, car_length):
    offset = car_length * (0.5 - REAR_AXLE_RATIO)
    return (cx - offset * math.cos(heading), cy - offset * math.sin(heading))


def _find_closest_idx(path, x, y, start):
    best_d = float("inf")
    best_i = start
    for i in range(start, len(path)):
        d = (path[i][0] - x) ** 2 + (path[i][1] - y) ** 2
        if d < best_d:
            best_d = d
            best_i = i
        elif i > best_i + 5:
            break
    return best_i


def _find_entry_index(cx, cy, heading, centerline):
    """Find the best entry index on the centerline for a bezier from (cx,cy).

    Balances turn sharpness (lateral/forward ratio) against detour length.
    Returns 0 when the car is already well-positioned for centerline[0].
    """
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    best_idx = 0
    best_score = float("inf")
    for i, pt in enumerate(centerline):
        dx = pt[0] - cx
        dy = pt[1] - cy
        forward = dx * cos_h + dy * sin_h
        if forward <= 0.5:
            continue  # skip points behind / barely ahead
        lateral = abs(-dx * sin_h + dy * cos_h)
        # Penalize sharp turns (high lateral/forward) and long detours
        score = lateral * lateral / forward + 0.3 * forward
        if score < best_score:
            best_score = score
            best_idx = i
    return best_idx


def _find_look_ahead(path, idx, look_ahead_dist):
    remaining = look_ahead_dist
    i = idx
    while i < len(path) - 1:
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        seg = math.hypot(dx, dy)
        if remaining <= seg and seg > 0.001:
            t = remaining / seg
            return (path[i][0] + t * dx, path[i][1] + t * dy)
        remaining -= seg
        i += 1
    return path[-1]


def _pure_pursuit(rx, ry, heading, gx, gy, wheelbase, max_steer):
    dx = gx - rx
    dy = gy - ry
    ld = math.hypot(dx, dy)
    if ld < 0.01:
        return 0
    alpha = math.atan2(dy, dx) - heading
    steer = math.atan(2 * wheelbase * math.sin(alpha) / ld)
    return max(-max_steer, min(max_steer, steer))


def _reverse_pursuit(rx, ry, heading, gx, gy, wheelbase, max_steer):
    """Pure pursuit for reverse driving.

    Computes steering to move the rear axle toward (gx, gy) while
    driving backward. The target should be in the backward direction.
    """
    dx = gx - rx
    dy = gy - ry
    ld = math.hypot(dx, dy)
    if ld < 0.01:
        return 0
    # Angle from backward direction to target
    backward = heading + math.pi
    alpha = math.atan2(dy, dx) - backward
    # Normalize to [-pi, pi]
    alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
    # Reverse steering: positive alpha → steer right to swing rear left
    steer = -math.atan(2 * wheelbase * math.sin(alpha) / ld)
    return max(-max_steer, min(max_steer, steer))


def _simulate_reverse(
    start_center: tuple[float, float],
    start_heading: float,
    path: list[tuple[float, float]],
    car_length: float,
    car_width: float,
    maneuvering: Polygon,
    other_cars: list[Polygon],
    building: Polygon | None = None,
) -> list[tuple[float, float, float]]:
    """Simulate bicycle model REVERSING along path with boundary enforcement.

    The car drives backward along the guide path (rear follows path).
    When stuck, a short forward bump creates room.
    Returns trajectory as list of (center_x, center_y, heading_rad) in local meters.
    """
    wheelbase = car_length * WHEELBASE_FACTOR
    max_steer = math.radians(MAX_STEER_DEG)
    look_ahead = wheelbase * LOOK_AHEAD_FACTOR

    rx, ry = _center_to_rear(*start_center, start_heading, car_length)
    h = start_heading
    path_idx = 0
    trajectory: list[tuple[float, float, float]] = []

    bumping = False
    bump_left = 0
    best_dist = float("inf")
    stuck_counter = 0
    STUCK_LIMIT = 600

    for step in range(MAX_SIM_STEPS):
        cx, cy = _rear_to_center(rx, ry, h, car_length)
        trajectory.append((cx, cy, h))

        # Check arrival at path end
        end = path[-1]
        dist_to_end = math.hypot(rx - end[0], ry - end[1])
        if dist_to_end < ARRIVAL_DIST:
            break

        # Stuck detection
        if dist_to_end < best_dist - 0.1:
            best_dist = dist_to_end
            stuck_counter = 0
        else:
            stuck_counter += 1
            if stuck_counter >= STUCK_LIMIT:
                break

        if bumping:
            # Forward bump to create room when stuck in reverse
            path_idx_b = _find_closest_idx(path, rx, ry, max(0, path_idx - 10))
            target_b = _find_look_ahead(path, path_idx_b, look_ahead * 2)
            # Steer front toward path target during bump
            steer = _pure_pursuit(rx, ry, h, target_b[0], target_b[1], wheelbase, max_steer)

            v = SPEED * BUMP_SPEED_FACTOR
            new_rx = rx + v * math.cos(h) * DT
            new_ry = ry + v * math.sin(h) * DT
            new_h = h + (v * math.tan(steer) / wheelbase) * DT

            rect = _car_rect_from_rear(new_rx, new_ry, new_h, car_length, car_width)
            bump_inside = (
                maneuvering.intersection(rect).area / rect.area
                if rect.area > 0
                else 0
            )
            hits = any(rect.intersects(obs) for obs in other_cars)
            bldg_ovlp = (
                building.intersection(rect).area / rect.area
                if building is not None and rect.area > 0 else 0
            )
            if bump_inside >= MIN_INSIDE_RATIO and not hits and bldg_ovlp <= MAX_BUILDING_OVERLAP:
                rx, ry, h = new_rx, new_ry, new_h

            bump_left -= 1
            if bump_left <= 0:
                bumping = False
            continue

        # Reverse: steer rear toward the path target
        path_idx = _find_closest_idx(path, rx, ry, path_idx)
        target = _find_look_ahead(path, path_idx, look_ahead)
        steer = _reverse_pursuit(
            rx, ry, h, target[0], target[1], wheelbase, max_steer
        )

        v = -SPEED
        new_rx = rx + v * math.cos(h) * DT
        new_ry = ry + v * math.sin(h) * DT
        new_h = h + (v * math.tan(steer) / wheelbase) * DT

        # Check bounds
        rect = _car_rect_from_rear(new_rx, new_ry, new_h, car_length, car_width)
        inside_ratio = (
            maneuvering.intersection(rect).area / rect.area
            if rect.area > 0
            else 0
        )
        hits_car = any(rect.intersects(obs) for obs in other_cars)
        bldg_overlap = (
            building.intersection(rect).area / rect.area
            if building is not None and rect.area > 0 else 0
        )

        if inside_ratio >= MIN_INSIDE_RATIO and not hits_car and bldg_overlap <= MAX_BUILDING_OVERLAP:
            rx, ry, h = new_rx, new_ry, new_h
        else:
            # Can't reverse — forward bump to create room
            bumping = True
            bump_left = BUMP_STEPS

    return trajectory


def _simulate_forward(
    start_center: tuple[float, float],
    start_heading: float,
    path: list[tuple[float, float]],
    car_length: float,
    car_width: float,
    maneuvering: Polygon,
    other_cars: list[Polygon],
    building: Polygon | None = None,
) -> list[tuple[float, float, float]]:
    """Simulate bicycle model driving FORWARD along path with boundary enforcement.

    Includes reverse-bump capability for multi-point turns when the car
    gets stuck at tight corners.
    """
    wheelbase = car_length * WHEELBASE_FACTOR
    max_steer = math.radians(MAX_STEER_DEG)
    look_ahead = wheelbase * LOOK_AHEAD_FACTOR

    rx, ry = _center_to_rear(*start_center, start_heading, car_length)
    h = start_heading
    path_idx = 0
    trajectory: list[tuple[float, float, float]] = []

    bumping = False
    bump_left = 0
    best_dist = float("inf")
    stuck_counter = 0
    STUCK_LIMIT = 600

    for step in range(MAX_SIM_STEPS):
        cx, cy = _rear_to_center(rx, ry, h, car_length)
        trajectory.append((cx, cy, h))

        end = path[-1]
        dist_to_end = math.hypot(rx - end[0], ry - end[1])
        if dist_to_end < ARRIVAL_DIST:
            break

        if dist_to_end < best_dist - 0.1:
            best_dist = dist_to_end
            stuck_counter = 0
        else:
            stuck_counter += 1
            if stuck_counter >= STUCK_LIMIT:
                break

        if bumping:
            # Reverse bump to create room when stuck going forward
            steer = _reverse_pursuit(
                rx, ry, h, path[max(0, path_idx - 3)][0],
                path[max(0, path_idx - 3)][1], wheelbase, max_steer
            )
            v = -SPEED * BUMP_SPEED_FACTOR
            new_rx = rx + v * math.cos(h) * DT
            new_ry = ry + v * math.sin(h) * DT
            new_h = h + (v * math.tan(steer) / wheelbase) * DT

            rect = _car_rect_from_rear(new_rx, new_ry, new_h, car_length, car_width)
            bump_ok = (
                maneuvering.intersection(rect).area / rect.area >= MIN_INSIDE_RATIO
                if rect.area > 0 else False
            )
            hits = any(rect.intersects(obs) for obs in other_cars)
            bldg_bump = (
                building.intersection(rect).area / rect.area
                if building is not None and rect.area > 0 else 0
            )
            if bump_ok and not hits and bldg_bump <= MAX_BUILDING_OVERLAP:
                rx, ry, h = new_rx, new_ry, new_h

            bump_left -= 1
            if bump_left <= 0:
                bumping = False
            continue

        path_idx = _find_closest_idx(path, rx, ry, path_idx)
        target = _find_look_ahead(path, path_idx, look_ahead)
        steer = _pure_pursuit(rx, ry, h, target[0], target[1], wheelbase, max_steer)

        v = SPEED
        new_rx = rx + v * math.cos(h) * DT
        new_ry = ry + v * math.sin(h) * DT
        new_h = h + (v * math.tan(steer) / wheelbase) * DT

        rect = _car_rect_from_rear(new_rx, new_ry, new_h, car_length, car_width)
        inside_ratio = (
            maneuvering.intersection(rect).area / rect.area
            if rect.area > 0 else 0
        )
        hits_car = any(rect.intersects(obs) for obs in other_cars)
        bldg_overlap = (
            building.intersection(rect).area / rect.area
            if building is not None and rect.area > 0 else 0
        )

        if inside_ratio >= MIN_INSIDE_RATIO and not hits_car and bldg_overlap <= MAX_BUILDING_OVERLAP:
            rx, ry, h = new_rx, new_ry, new_h
        else:
            # Can't go forward — reverse bump to create room
            bumping = True
            bump_left = BUMP_STEPS

    return trajectory



def compute_drive_out_paths(
    placements: list[CarPlacement],
) -> list[DriveOutPath]:
    """Compute drive-out trajectories for all placed cars.

    Each car's trajectory is pre-computed using a bicycle model with boundary
    enforcement. Cars leave closest-to-exit first.
    """
    if not placements:
        return []

    parking_m = polygon_to_local(get_parking_area_wgs84())
    row_m = polygon_to_local(get_right_of_way_polygon_wgs84())
    building_m = polygon_to_local(get_building_polygon_wgs84())
    # Buffer allows minor overhangs during tight turns, but exclude building
    maneuvering = parking_m.union(row_m).buffer(0.5).difference(building_m)

    car_length, car_width = CAR_DIMENSIONS[placements[0].car_type]

    # Compute safe centerline (eroded by half car width)
    centerline = _compute_safe_centerline(row_m, parking_m, car_width)

    # Compute turning waypoint in upper ROW (far from corridor exit)
    row_coords = list(row_m.exterior.coords[:-1])
    corridor_exit = centerline[-1] if centerline else (row_m.centroid.x, row_m.centroid.y)
    by_dist = sorted(
        row_coords,
        key=lambda c: math.hypot(c[0] - corridor_exit[0], c[1] - corridor_exit[1]),
        reverse=True,
    )
    turn_wp = (
        sum(c[0] for c in by_dist[:3]) / 3,
        sum(c[1] for c in by_dist[:3]) / 3,
    )
    if not maneuvering.contains(Point(turn_wp)):
        turn_wp = (row_m.centroid.x, row_m.centroid.y)
    turn_wp_pt = Point(turn_wp)

    # Build obstacle rects (buffered for path planning, raw for driving sim)
    car_rects_buffered: dict[int, Polygon] = {}
    car_rects_raw: dict[int, Polygon] = {}
    for p in placements:
        raw = _build_car_rect(p)
        car_rects_raw[p.car_index] = raw
        car_rects_buffered[p.car_index] = raw.buffer(OBSTACLE_BUFFER)

    # Sort: closest to corridor entrance goes first (clears path for cars behind)
    corridor_entrance_pt = Point(centerline[0])

    def _sort_key(p):
        car_pos = Point(wgs84_to_local(*p.center))
        return car_pos.distance(corridor_entrance_pt)

    sorted_placements = sorted(placements, key=_sort_key)

    driven_out: set[int] = set()
    paths: list[DriveOutPath] = []

    for placement in sorted_placements:
        car_local = wgs84_to_local(*placement.center)
        heading_rad = math.radians(placement.heading)

        obstacles_buffered = [
            car_rects_buffered[idx]
            for idx in car_rects_buffered
            if idx != placement.car_index and idx not in driven_out
        ]
        obstacles_raw = [
            car_rects_raw[idx]
            for idx in car_rects_raw
            if idx != placement.car_index and idx not in driven_out
        ]

        # Corridor direction from first centerline points
        if len(centerline) >= 3:
            corridor_dir = math.atan2(
                centerline[2][1] - centerline[0][1],
                centerline[2][0] - centerline[0][0],
            )
        else:
            corridor_dir = math.atan2(
                centerline[-1][1] - centerline[0][1],
                centerline[-1][0] - centerline[0][0],
            )

        # Direction from turn_wp toward corridor entrance
        dir_to_corridor = math.atan2(
            centerline[0][1] - turn_wp[1],
            centerline[0][0] - turn_wp[0],
        )

        # Check if car faces roughly toward exit or away
        exit_dir = math.atan2(
            corridor_exit[1] - car_local[1],
            corridor_exit[0] - car_local[0],
        )
        heading_vs_exit = abs(
            (heading_rad - exit_dir + math.pi) % (2 * math.pi) - math.pi
        )
        faces_exit = heading_vs_exit < math.pi / 2

        if faces_exit:
            # Shrink obstacle rects to allow close passes during maneuvering
            driving_obstacles = [o.buffer(-0.10) for o in obstacles_raw]

            # Pre-reverse only when heading points toward building
            bldg_ahead = Point(
                car_local[0] + car_length * math.cos(heading_rad),
                car_local[1] + car_length * math.sin(heading_rad),
            )
            needs_reverse = building_m.distance(bldg_ahead) < car_width

            traj_reverse: list[tuple[float, float, float]] = []
            reverse_path: list[tuple[float, float]] = []
            rev_cx, rev_cy = car_local

            if needs_reverse:
                backward_dir = heading_rad + math.pi
                for d_cm in range(25, 501, 25):
                    d = d_cm / 100.0
                    cx = car_local[0] + d * math.cos(backward_dir)
                    cy = car_local[1] + d * math.sin(backward_dir)
                    rx, ry = _center_to_rear(cx, cy, heading_rad, car_length)
                    rect = _car_rect_from_rear(
                        rx, ry, heading_rad, car_length, car_width
                    )
                    inside = (
                        maneuvering.intersection(rect).area / rect.area
                        if rect.area > 0 else 0
                    )
                    bldg = (
                        building_m.intersection(rect).area / rect.area
                        if rect.area > 0 else 0
                    )
                    hits = any(
                        rect.intersects(obs) for obs in driving_obstacles
                    )
                    if (
                        inside >= MIN_INSIDE_RATIO
                        and bldg <= MAX_BUILDING_OVERLAP
                        and not hits
                    ):
                        rev_cx, rev_cy = cx, cy
                    else:
                        break

                rev_dist = math.hypot(
                    rev_cx - car_local[0], rev_cy - car_local[1]
                )
                if rev_dist > 0.1:
                    rev_steps = max(1, int(rev_dist / (SPEED * DT)))
                    for i in range(rev_steps + 1):
                        t = min(i / max(rev_steps, 1), 1.0)
                        cx = car_local[0] + t * rev_dist * math.cos(
                            backward_dir
                        )
                        cy = car_local[1] + t * rev_dist * math.sin(
                            backward_dir
                        )
                        traj_reverse.append((cx, cy, heading_rad))
                    reverse_path = [
                        (cx, cy) for cx, cy, _ in traj_reverse
                    ]

            last_cx, last_cy = rev_cx, rev_cy
            last_h = heading_rad

            # Phase 2: Forward curve to corridor
            # Find best entry point on centerline (avoids sharp turns)
            p0 = (last_cx, last_cy)
            start_h = last_h
            entry_idx = _find_entry_index(p0[0], p0[1], start_h, centerline)
            p3 = centerline[entry_idx]
            remaining_cl = list(centerline[entry_idx + 1:])
            d1 = math.hypot(p3[0] - p0[0], p3[1] - p0[1])

            # Corridor direction at the entry point
            if entry_idx + 2 < len(centerline):
                entry_dir = math.atan2(
                    centerline[entry_idx + 2][1] - p3[1],
                    centerline[entry_idx + 2][0] - p3[0],
                )
            else:
                entry_dir = corridor_dir
            reverse_entry = entry_dir + math.pi

            # Tight bezier from current (possibly reversed) position
            tl = max(2.0, d1 * 0.45)
            # Check if heading points toward building — offset P1 away
            p1_dir = start_h
            ahead_pt = Point(
                p0[0] + car_length * math.cos(start_h),
                p0[1] + car_length * math.sin(start_h),
            )
            if building_m.distance(ahead_pt) < car_width:
                # Building is close ahead — steer P1 away from it
                bldg_near = building_m.exterior.interpolate(
                    building_m.exterior.project(Point(p0))
                )
                left = (-math.sin(start_h), math.cos(start_h))
                bx, by = bldg_near.x - p0[0], bldg_near.y - p0[1]
                bldg_on_left = bx * left[0] + by * left[1] > 0
                p1_dir = start_h + (-0.35 if bldg_on_left else 0.35)
            p1 = (p0[0] + tl * math.cos(p1_dir),
                  p0[1] + tl * math.sin(p1_dir))
            p2 = (p3[0] + tl * math.cos(reverse_entry),
                  p3[1] + tl * math.sin(reverse_entry))
            transition = _cubic_bezier(p0, p1, p2, p3, n_points=20)

            forward_path = _densify(
                transition + remaining_cl,
                DENSIFY_SPACING,
            )
            forward_path = _reroute_around_building(forward_path, building_m, maneuvering)

            traj_forward = _simulate_forward(
                p0, start_h, forward_path,
                car_length, car_width, maneuvering, driving_obstacles,
                building=building_m,
            )

            # Check if tight bezier got stuck
            fwd_end = traj_forward[-1]
            fwd_end_dist = math.hypot(
                fwd_end[0] - forward_path[-1][0],
                fwd_end[1] - forward_path[-1][1],
            )

            if fwd_end_dist > ARRIVAL_DIST * 2 and driving_obstacles:
                # Tight bezier stuck — try wide bezier to sweep around obstacles
                tl_w = max(car_length, d1 * 0.7)
                p1w = (p0[0] + tl_w * math.cos(start_h),
                       p0[1] + tl_w * math.sin(start_h))
                p2w = (p3[0] + tl_w * math.cos(reverse_entry),
                       p3[1] + tl_w * math.sin(reverse_entry))
                transition_w = _cubic_bezier(p0, p1w, p2w, p3, n_points=30)

                wide_fwd_path = _densify(
                    transition_w + remaining_cl,
                    DENSIFY_SPACING,
                )
                wide_fwd_path = _reroute_around_building(wide_fwd_path, building_m, maneuvering)

                traj_wide = _simulate_forward(
                    p0, start_h, wide_fwd_path,
                    car_length, car_width, maneuvering, driving_obstacles,
                    building=building_m,
                )

                # Pick whichever traveled farther
                tight_travel = math.hypot(fwd_end[0] - p0[0], fwd_end[1] - p0[1])
                wide_end = traj_wide[-1]
                wide_travel = math.hypot(wide_end[0] - p0[0], wide_end[1] - p0[1])

                if wide_travel > tight_travel:
                    traj_forward = traj_wide
                    forward_path = wide_fwd_path

            traj_local = traj_reverse + traj_forward[1:]
            guide_path = reverse_path + forward_path[1:]
        else:
            # --- Car faces away from exit: reverse to clear, then forward ---
            backward_dir = heading_rad + math.pi

            # Phase 1: Brief reverse to clear building corner
            clear_dist = 5.0
            clear_pt = (
                car_local[0] + clear_dist * math.cos(backward_dir),
                car_local[1] + clear_dist * math.sin(backward_dir),
            )
            reverse_path = _densify(
                [car_local, clear_pt], DENSIFY_SPACING
            )

            traj_reverse = _simulate_reverse(
                car_local, heading_rad, reverse_path,
                car_length, car_width, maneuvering, obstacles_raw,
                building=building_m,
            )

            last_cx, last_cy, last_h = traj_reverse[-1]

            # Phase 2: Forward from cleared position through upper ROW to Shore Rd
            p0 = (last_cx, last_cy)
            p3 = turn_wp
            d1 = math.hypot(p3[0] - p0[0], p3[1] - p0[1])
            tl1 = max(2.0, d1 * 0.45)
            p1 = (p0[0] + tl1 * math.cos(last_h),
                  p0[1] + tl1 * math.sin(last_h))
            p2 = (p3[0] - tl1 * math.cos(dir_to_corridor),
                  p3[1] - tl1 * math.sin(dir_to_corridor))

            p0b = turn_wp
            p3b = centerline[0]
            d2 = math.hypot(p3b[0] - p0b[0], p3b[1] - p0b[1])
            tl2 = max(2.0, d2 * 0.45)
            p1b = (p0b[0] + tl2 * math.cos(dir_to_corridor),
                   p0b[1] + tl2 * math.sin(dir_to_corridor))
            reverse_corridor = corridor_dir + math.pi
            p2b = (p3b[0] + tl2 * math.cos(reverse_corridor),
                   p3b[1] + tl2 * math.sin(reverse_corridor))

            raw_fwd = (
                _cubic_bezier(p0, p1, p2, p3, n_points=20) +
                _cubic_bezier(p0b, p1b, p2b, p3b, n_points=20)[1:] +
                list(centerline[1:])
            )
            forward_path = _densify(raw_fwd, DENSIFY_SPACING)
            forward_path = _reroute_around_building(forward_path, building_m, maneuvering)

            traj_forward = _simulate_forward(
                (last_cx, last_cy), last_h, forward_path,
                car_length, car_width, maneuvering, obstacles_raw,
                building=building_m,
            )

            traj_local = traj_reverse + traj_forward[1:]
            guide_path = reverse_path + forward_path[1:]

        # Convert to WGS84 for API response
        waypoints_wgs84 = [list(local_to_wgs84(x, y)) for x, y in guide_path]

        # Thin trajectory (every FRAME_SKIP-th frame)
        traj_wgs84: list[list[float]] = []
        for i in range(0, len(traj_local), FRAME_SKIP):
            cx, cy, h = traj_local[i]
            lon, lat = local_to_wgs84(cx, cy)
            traj_wgs84.append([lon, lat, math.degrees(h)])
        # Always include last frame
        if traj_local:
            cx, cy, h = traj_local[-1]
            lon, lat = local_to_wgs84(cx, cy)
            last = [lon, lat, math.degrees(h)]
            if traj_wgs84[-1] != last:
                traj_wgs84.append(last)

        paths.append(
            DriveOutPath(
                car_index=placement.car_index,
                waypoints=waypoints_wgs84,
                trajectory=traj_wgs84,
            )
        )

        driven_out.add(placement.car_index)

    return paths
