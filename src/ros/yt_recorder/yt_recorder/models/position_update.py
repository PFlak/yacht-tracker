from pydantic import BaseModel, Field
from typing import Literal, Optional

from yt_recorder.models.boat_position import BoatPosition


class PositionUpdate(BaseModel):
    boat_id: int
    timestamp: float
    boat_position: BoatPosition
    status: Optional[Literal['ok', 'error']] = Field(None)
