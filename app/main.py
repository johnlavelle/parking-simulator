"""FastAPI application serving the parking simulator."""

from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from app.geometry import (
    get_building_polygon_wgs84,
    get_parking_area_wgs84,
    get_right_of_way_polygon_wgs84,
    get_row_min_width,
    polygon_to_geojson,
)
from app.models import (
    SimulateRequest, SimulateResponse, PolygonsResponse,
    CAR_DIMENSIONS, CarType,
)
from app.optimizer import optimize_placement
from app.pathfinder import compute_drive_out_paths

app = FastAPI(title="Parking Simulator - 29 Shore Road")

# Maneuvering margin: car width must be at most ROW width minus this
ROW_WIDTH_MARGIN = 0.15  # meters (each side ~7.5cm)


@app.get("/api/polygons", response_model=PolygonsResponse)
def get_polygons():
    return PolygonsResponse(
        building=polygon_to_geojson(get_building_polygon_wgs84()),
        parking_area=polygon_to_geojson(get_parking_area_wgs84()),
        right_of_way=polygon_to_geojson(get_right_of_way_polygon_wgs84()),
    )


@app.get("/api/config")
def get_config():
    """Return allowed car types (those that fit through the narrow ROW) and limits."""
    min_width = get_row_min_width()
    max_car_width = min_width - ROW_WIDTH_MARGIN
    allowed = []
    for ct in CarType:
        length, width = CAR_DIMENSIONS[ct]
        if width <= max_car_width:
            allowed.append({
                "value": ct.value,
                "label": f"{ct.value} ({length:.2f} × {width:.2f}m)",
                "length": length,
                "width": width,
            })
    return {
        "max_cars": 4,
        "row_min_width": round(min_width, 2),
        "max_car_width": round(max_car_width, 2),
        "car_types": allowed,
    }


@app.post("/api/simulate", response_model=SimulateResponse)
def simulate(req: SimulateRequest):
    car_length, car_width = CAR_DIMENSIONS[req.car_type]
    min_width = get_row_min_width()
    if car_width > min_width - ROW_WIDTH_MARGIN:
        raise HTTPException(
            status_code=400,
            detail=f"{req.car_type.value} ({car_width:.2f}m wide) does not fit "
                   f"through the narrowest ROW section ({min_width:.2f}m wide)",
        )
    placements, warnings = optimize_placement(req.num_cars, req.car_type)
    paths = compute_drive_out_paths(placements)
    return SimulateResponse(
        placements=placements,
        paths=paths,
        warnings=warnings,
        car_length=car_length,
        car_width=car_width,
    )


# Serve static files
app.mount("/static", StaticFiles(directory="static"), name="static")


@app.get("/")
def index():
    return FileResponse("static/index.html")
