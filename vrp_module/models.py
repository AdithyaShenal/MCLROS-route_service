from pydantic import BaseModel
from typing import List, Tuple

class AutoRequest(BaseModel):
    coords: List[Tuple[float, float]]
    demands: List[int]
    vehicle_capacities: List[int]

class RouteData(BaseModel):
    route: int
    coords: List[Tuple[float, float]]
    demands: List[int]
    vehicle_capacities: List[int]

class RouteWiseAllRequest(BaseModel):
    requestBody: List[RouteData]