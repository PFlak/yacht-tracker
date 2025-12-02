from pydantic import BaseModel

class UserRegister(BaseModel):
    username: str
    email: str
    password: str
    phone: str | None = None


class User(BaseModel):
    id: int
    username: str
    email: str
    phone: str | None
    created_at: float
