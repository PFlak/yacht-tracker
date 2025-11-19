from pydantic import BaseModel, Field

from yt_recorder.models.boat_position import BoatPosition


class BoatRoute(BaseModel):
    boat_id: int
    positions: list[BoatPosition] = Field([])
