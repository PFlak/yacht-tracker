from fastapi.responses import JSONResponse, HTMLResponse
from fastapi import status, Request
from fastapi.routing import APIRouter
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from .ui_recorder import recorder_router
from ..models.boats import Boats, Boat
from ..utils.time import generate_random_timestamp
import random
import os
from datetime import datetime
from ..db import get_db_cursor

ui_router = APIRouter()

ui_router.include_router(recorder_router, prefix='/recorder')

#frontend
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TEMPLATES_DIR = os.path.join(BASE_DIR, "frontend", "templates")
STATIC_DIR = os.path.join(BASE_DIR, "frontend", "static")

templates = Jinja2Templates(directory=TEMPLATES_DIR)

@ui_router.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})


@ui_router.get("/login", response_class=HTMLResponse)
async def login(request: Request):
    return templates.TemplateResponse("login.html", {"request": request})


@ui_router.get("/register", response_class=HTMLResponse)
async def register(request: Request):
    return templates.TemplateResponse("register.html", {"request": request})


@ui_router.get("/my_fleet", response_class=HTMLResponse)
async def my_fleet(request: Request):
    return templates.TemplateResponse("my_fleet.html", {"request": request})


@ui_router.get("/incidents", response_class=HTMLResponse)
async def incidents(request: Request):
    return templates.TemplateResponse("incidents.html", {"request": request})


@ui_router.get("/recorders", response_model=Boats)
def get_all_recorders():
    boats: list[Boat] = []

    with get_db_cursor() as cur:
        cur.execute(
            """
            SELECT yi.yacht_id,
                   yi.email,
                   yi.phone,
                   MAX(yp.timestamp) AS last_ts
            FROM dbo.YachtInfo AS yi
            LEFT JOIN dbo.YachtPosition AS yp
                ON yp.yacht_id = yi.yacht_id
            GROUP BY yi.yacht_id, yi.email, yi.phone
            ORDER BY yi.yacht_id
            """
        )
        rows = cur.fetchall()

    now_ts = datetime.now().timestamp()

    for row in rows:
        yacht_id, email, phone, last_ts = row

        last_seen_ts: float | None = None
        if last_ts is not None:
            last_seen_ts = last_ts.timestamp()

        is_online = False
        if last_seen_ts is not None and (now_ts - last_seen_ts) < 5 * 60:
            is_online = True

        boat = Boat(
            id=int(yacht_id),
            name=email or f"Yacht {yacht_id}",
            created_at=now_ts,
            is_online=is_online,
            is_registered=True,
            last_seen_at=last_seen_ts,
            model=None,
            boat_type="other",
            contact_number=phone,
        )
        boats.append(boat)

    bs = Boats(array=boats)

    return JSONResponse(bs.model_dump(), status_code=status.HTTP_200_OK)
