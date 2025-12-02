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


@ui_router.get('/recorders', response_model=Boats)
def get_all_recorders():
    b1 = Boat(id=random.randint(1, 100),
              name=f'boat-{random.randint(1,100)}',
              created_at=generate_random_timestamp().timestamp(),
              is_online=False,
              is_registered=True,
              last_seen_at=generate_random_timestamp().timestamp(),
              model="Antila 27",
              boat_type='sailboat',
              contact_number='666666666')

    b2 = Boat(id=random.randint(1, 100),
              name=f'boat-{random.randint(1,100)}',
              created_at=generate_random_timestamp().timestamp(),
              is_online=False,
              is_registered=True,
              last_seen_at=generate_random_timestamp().timestamp(),
              model="Maxis 33.3",
              boat_type='motorboat',
              contact_number='666666666')

    b3 = Boat(id=random.randint(1, 100),
              name=f'boat-{random.randint(1,100)}',
              created_at=generate_random_timestamp().timestamp(),
              is_online=False,
              is_registered=True,
              last_seen_at=generate_random_timestamp().timestamp(),
              model="Maxis 33.3",
              boat_type='motorboat',
              contact_number='666666666')

    bs = Boats(array=[b1, b2, b3])

    response = JSONResponse(bs.model_dump(),
                            status_code=status.HTTP_200_OK)

    return response
