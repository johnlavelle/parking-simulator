"""Pydantic request/response models."""

from enum import Enum
from pydantic import BaseModel, Field


class CarType(str, Enum):
    SMALL = "Small"
    MEDIUM = "Medium"
    LARGE = "Large"
    SUV = "SUV"


# Car dimensions in meters: (length, width)
CAR_DIMENSIONS: dict[CarType, tuple[float, float]] = {
    CarType.SMALL: (3.57, 1.63),
    CarType.MEDIUM: (4.30, 1.80),
    CarType.LARGE: (4.94, 1.87),
    CarType.SUV: (5.00, 2.07),
}


class SimulateRequest(BaseModel):
    num_cars: int = Field(ge=1, le=4, default=2)
    car_type: CarType = CarType.MEDIUM


class CarPlacement(BaseModel):
    car_index: int
    center: list[float]  # [lon, lat]
    corners: list[list[float]]  # [[lon, lat], ...]
    angle: float  # degrees (rectangle orientation)
    heading: float  # degrees (direction front of car faces, from east CCW)
    car_type: CarType


class DriveOutPath(BaseModel):
    car_index: int
    waypoints: list[list[float]]  # [[lon, lat], ...] for path line display
    trajectory: list[list[float]]  # [[lon, lat, heading_deg], ...] pre-computed frames


class SimulateResponse(BaseModel):
    placements: list[CarPlacement]
    paths: list[DriveOutPath]
    warnings: list[str] = []
    car_length: float
    car_width: float


class PolygonsResponse(BaseModel):
    building: dict
    parking_area: dict
    right_of_way: dict
