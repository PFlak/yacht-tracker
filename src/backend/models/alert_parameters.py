from pydantic import BaseModel, Field
from typing import Optional

from .boat_position import BoatPosition


class AlertParameters(BaseModel):
    id: int
    alert_id: int
    timestamp: float
    boat_position: BoatPosition
    boat_speed: Optional[float] = Field(None)
    windspeed: Optional[float] = Field(None)
