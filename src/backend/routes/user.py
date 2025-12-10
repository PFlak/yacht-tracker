from fastapi import APIRouter, HTTPException
from ..models.user import UserRegister, User
from ..db import get_db_cursor
import hashlib

user_router = APIRouter()


@user_router.post("/register", response_model=User)
def register_user(body: UserRegister):

    password_hash = hashlib.sha256(body.password.encode("utf-8")).digest()

    with get_db_cursor() as cur:

        cur.execute("SELECT id FROM Users WHERE email = ?", (body.email,))
        if cur.fetchone():
            raise HTTPException(status_code=400, detail="Email already exists")

        cur.execute(
            """
            INSERT INTO Users (username, email, password_hash, phone)
            OUTPUT INSERTED.id, INSERTED.created_at
            VALUES (?, ?, ?, ?)
            """,
            (body.username, body.email, password_hash, body.phone),
        )

        row = cur.fetchone()
        user_id, created_at = row

    return User(
        id=user_id,
        username=body.username,
        email=body.email,
        phone=body.phone,
        created_at=created_at.timestamp(),
    )
