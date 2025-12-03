from pydantic import BaseModel, Field

from models.boat_position import BoatPosition


class BoatRoute(BaseModel):
    boat_id: int
    positions: list[BoatPosition] = Field([])
