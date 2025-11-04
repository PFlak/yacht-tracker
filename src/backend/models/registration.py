from pydantic import BaseModel, Field
from typing import Optional, Literal


class Registration(BaseModel):
    registration_code: str
    boat_id: Optional[int] = Field(None)
    status: Literal['ok', 'error'] = Field(None)
