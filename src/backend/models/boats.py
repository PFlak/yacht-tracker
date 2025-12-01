from pydantic import BaseModel, Field
from typing import Literal, Optional


class NewBoat(BaseModel):
    name: str
    created_at: float
    boat_type: Optional[Literal['sailboat', 'motorboat', 'other']] = Field(None)
    model: Optional[str] = Field(None)
    contact_number: Optional[str] = Field(None)


class Boat(BaseModel):
    id: int
    name: str
    created_at: float
    deleted_flag: bool = Field(False, description='wether boat is deleted')
    is_online: bool
    is_registered: bool
    last_seen_at: Optional[float] = Field(None, description='Last moment when recorder was online')
    model: Optional[str] = Field(None)
    boat_type: Optional[Literal['sailboat', 'motorboat', 'other']] = Field(None)
    contact_number: Optional[str] = Field(None, description='Contact number of the current boat user')


class UpdateBoat(BaseModel):
    status: Literal['ok', 'error']
    message: Optional[str] = Field(None, description='Message when error status occurs')
    boat: Optional[Boat] = Field(None, description='Updated boat if no error occurred')


class Boats(BaseModel):
    array: list[Boat]
