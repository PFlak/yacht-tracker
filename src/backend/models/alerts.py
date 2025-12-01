from typing import Literal, Optional
from pydantic import BaseModel, Field

from .alert_parameters import AlertParameters


class Alert(BaseModel):
    id: int
    boat_id: int
    timestamp: float
    alert_type: Literal['danger', 'weather', 'other']
    severity: int = Field(0, ge=0, le=3)
    description: str
    parameters: list[AlertParameters] = Field([])
    latitude: Optional[float] = Field(None)
    longitude: Optional[float] = Field(None)
    resolved: Optional[bool] = Field(False)


class Alerts(BaseModel):
    data: list[Alert]
