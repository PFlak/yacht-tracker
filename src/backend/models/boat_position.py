from pydantic import BaseModel


class BoatPosition(BaseModel):
    id: int
    recorder_id: int
    timestamp: float
    latitude: float
    longitude: float
