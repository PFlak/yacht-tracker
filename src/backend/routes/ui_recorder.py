from fastapi.routing import APIRouter
from fastapi import Path, Query, status, Body
from fastapi.responses import JSONResponse
from typing import Annotated
from ..models.boats import NewBoat, Boat, UpdateBoat
from ..models.boat_route import BoatRoute
from ..models.alerts import Alert, Alerts
from ..models.alert_parameters import AlertParameters
from ..utils.time import generate_random_timestamp, now_as_float
from ..utils.localization import generate_random_boat_position, generate_random_position
import random

recorder_router = APIRouter()


@recorder_router.post('/', response_model=Boat)
def create_recorder(body: NewBoat):
    b = Boat(id=random.randint(1, 100),
             name=f'boat-{random.randint(1, 100)}',
             created_at=body.created_at,
             is_online=False,
             is_registered=False,
             model=body.model,
             boat_type=body.boat_type,
             contact_number=body.contact_number)

    response = JSONResponse(b.model_dump(),
                            status_code=status.HTTP_200_OK)

    return response


@recorder_router.put('/', response_model=UpdateBoat)
def update_recorder(body: Boat):
    ub = UpdateBoat(status='ok',
                    boat=body)

    response = JSONResponse(ub.model_dump(),
                            status_code=status.HTTP_200_OK)

    return response


@recorder_router.get('/alerts', response_model=Alerts)
def get_recorders_incidents(last_h: int = 24):

    lt, lg = generate_random_position()

    a1 = Alert(id=1,
               boat_id=1,
               timestamp=generate_random_timestamp().timestamp(),
               alert_type=random.choice(['danger', 'weather', 'other']),
               severity=random.randint(0, 3),
               description="Lorem Ipsum",
               parameters=[AlertParameters(id=1,
                                           alert_id=1,
                                           timestamp=now_as_float(),
                                           boat_position=generate_random_boat_position())],
               latitude=lt,
               longitude=lg,
               resolved=True)

    a2 = Alert(id=2,
               boat_id=2,
               timestamp=generate_random_timestamp().timestamp(),
               alert_type=random.choice(['danger', 'weather', 'other']),
               severity=random.randint(0, 3),
               description="Lorem Ipsum",
               parameters=[AlertParameters(id=1,
                                           alert_id=1,
                                           timestamp=now_as_float(),
                                           boat_position=generate_random_boat_position())],
               latitude=lt,
               longitude=lg,
               resolved=True)

    alerts = Alerts(data=[a1, a2])

    response = JSONResponse(alerts.model_dump(),
                            status_code=status.HTTP_200_OK)

    return response


@recorder_router.get('/{recorder_id}/alerts', response_model=Alerts)
def get_recorder_incidents(recorder_id: int, last_h: int = 24, limit: int = 10):
    lt, lg = generate_random_position()

    a1 = Alert(id=random.randint(1, 100),
               boat_id=recorder_id,
               timestamp=generate_random_timestamp().timestamp(),
               alert_type=random.choice(['danger', 'weather', 'other']),
               severity=random.randint(0, 3),
               description="Lorem Ipsum",
               parameters=[AlertParameters(id=1,
                                           alert_id=1,
                                           timestamp=now_as_float(),
                                           boat_position=generate_random_boat_position())],
               latitude=lt,
               longitude=lg,
               resolved=True)

    a2 = Alert(id=random.randint(1, 100),
               boat_id=recorder_id,
               timestamp=generate_random_timestamp().timestamp(),
               alert_type=random.choice(['danger', 'weather', 'other']),
               severity=random.randint(0, 3),
               description="Lorem Ipsum",
               parameters=[AlertParameters(id=1,
                                           alert_id=1,
                                           timestamp=now_as_float(),
                                           boat_position=generate_random_boat_position())],
               latitude=lt,
               longitude=lg,
               resolved=True)

    alerts = Alerts(data=[a1, a2])

    response = JSONResponse(alerts.model_dump(),
                            status_code=status.HTTP_200_OK)
    return response


@recorder_router.get('{recorder_id}/route', response_model=BoatRoute)
def get_recorder_route(recorder_id: int,
                       history: bool = Query(description='wether to send position from the past', default=True),
                       depth: int = Query(default=60, description='Amount of route information in minutes')):

    br = BoatRoute(boat_id=recorder_id,
                   positions=[generate_random_boat_position() for i in range(history)])

    response = JSONResponse(br.model_dump(),
                            status_code=status.HTTP_200_OK)

    return response
