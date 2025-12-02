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
from datetime import datetime
from ..db import get_db_cursor
from ..models.boat_position import BoatPosition
from ..models.alerts import Alert, Alerts
from ..models.alert_parameters import AlertParameters
from ..utils.time import now_as_float

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


@recorder_router.get("/{recorder_id}/incidents", response_model=Alerts)
def get_recorders_incidents(
    recorder_id: int,
    last_h: int = 24,
):
    with get_db_cursor() as cur:
        cur.execute(
            """
            SELECT id, timestamp, event
            FROM dbo.YachtEvents
            WHERE yacht_id = ?
              AND timestamp >= DATEADD(hour, -?, GETDATE())
            ORDER BY timestamp DESC
            """,
            (recorder_id, last_h),
        )
        rows = cur.fetchall()

    alerts_list: list[Alert] = []

    for row in rows:
        event_id, ts, event_text = row
        ts_float = ts.timestamp() if isinstance(ts, datetime) else float(ts)

        alert = Alert(
            id=int(event_id),
            boat_id=recorder_id,
            timestamp=ts_float,
            alert_type="danger",
            severity=1,
            description=event_text,
            parameters=[],
            latitude=None,
            longitude=None,
            resolved=False,
        )
        alerts_list.append(alert)

    return JSONResponse(Alerts(data=alerts_list).model_dump(),
                        status_code=status.HTTP_200_OK)


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


@recorder_router.get("/{recorder_id}/route", response_model=BoatRoute)
def get_recorder_route(
    recorder_id: int,
    history: bool = Query(
        description="Czy zwrócić historię z przeszłości",
        default=True,
    ),
    depth: int = Query(
        default=60,
        description="Głębokość historii w minutach (ile wstecz)",
    ),
):
    with get_db_cursor() as cur:
        if history:
            cur.execute(
                """
                SELECT timestamp, latitude, longitude
                FROM dbo.YachtPosition
                WHERE yacht_id = ?
                  AND timestamp >= DATEADD(minute, -?, GETDATE())
                ORDER BY timestamp
                """,
                (recorder_id, depth),
            )
        else:
            cur.execute(
                """
                SELECT TOP 1 timestamp, latitude, longitude
                FROM dbo.YachtPosition
                WHERE yacht_id = ?
                ORDER BY timestamp DESC
                """,
                (recorder_id,),
            )
        rows = cur.fetchall()

    positions: list[BoatPosition] = []

    for idx, (ts, lat, lon) in enumerate(rows, start=1):
        ts_float = ts.timestamp() if isinstance(ts, datetime) else float(ts)
        positions.append(
            BoatPosition(
                id=idx,
                recorder_id=recorder_id,
                timestamp=ts_float,
                latitude=float(lat),
                longitude=float(lon),
            )
        )

    br = BoatRoute(boat_id=recorder_id, positions=positions)

    return JSONResponse(br.model_dump(), status_code=status.HTTP_200_OK)
