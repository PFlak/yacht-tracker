from pydantic import BaseModel
from typing import Optional


class BoatPosition(BaseModel):
    id: Optional[int]
    recorder_id: int
    timestamp: float
    latitude: float
    longitude: float
