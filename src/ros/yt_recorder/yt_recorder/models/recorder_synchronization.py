from pydantic import BaseModel, Field
from typing import Optional, Literal

from yt_recorder.models.boat_route import BoatRoute


class RecorderSynchronization(BaseModel):
    boat_id: int
    status: Literal['ok', 'error']
    timestamp: float
    sync_period: int = Field(30, description="Time in seconds in which synchronization request will be send")
    position_update_period: int = Field(30, description="Time in seconds in which new position should be published")
    position_stamp_period: int = Field(15, description="Time in seconds in which gps position stamp will be captured")
    boat_route: Optional[BoatRoute] = Field(None, description="Route gathered by recorder during period when it was online")
    direct_message: Optional[str] = Field(None, description="Message that should be shown on recorder display")
