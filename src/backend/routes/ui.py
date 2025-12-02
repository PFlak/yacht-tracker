from fastapi.responses import JSONResponse
from fastapi import status
from fastapi.routing import APIRouter
from .ui_recorder import recorder_router
from ..models.boats import Boats, Boat
from ..utils.time import generate_random_timestamp
import random

ui_router = APIRouter()

ui_router.include_router(recorder_router, prefix='/recorder')


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
