from pydantic import BaseModel, Field
from typing import Literal, Optional


class NewBoat(BaseModel):
    id: Optional[int] = Field(None)
    name: str
    created_at: float
    boat_type: Optional[Literal['sailboat', 'motorboat', 'other']] = Field(None)
    model: Optional[str] = Field(None)
    contact_number: Optional[str] = Field(None)


class Boat(BaseModel):
    id: int
    name: str
    created_at: float
    deleted_flag: bool = Field(False)
    is_online: bool
    is_registered: bool
    last_seen_at: Optional[float] = Field(None)
    model: Optional[str] = Field(None)
    boat_type: Optional[Literal['sailboat', 'motorboat', 'other']] = Field(None)
    contact_number: Optional[str] = Field(None)


class Boats(BaseModel):
    array: list[Boat]
