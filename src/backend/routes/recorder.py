from datetime import datetime
from fastapi import APIRouter, status
from fastapi.responses import JSONResponse

from ..models.registration import Registration
from ..models.recorder_synchronization import RecorderSynchronization
from ..models.position_update import PositionUpdate
from ..db import get_db_cursor

recorder_router = APIRouter()


@recorder_router.post("/register", response_model=Registration)
def recorder_registration(body: Registration):

    data = Registration(registration_code=body.registration_code,
                        boat_id=1,
                        status='ok')

    response = JSONResponse(content=data.model_dump(),
                            status_code=status.HTTP_200_OK)
    return response


@recorder_router.post("/{recorder_id}/sync", response_model=RecorderSynchronization)
def recorder_synchronization(recorder_id: int, body: RecorderSynchronization):
    now = datetime.now()

    data = RecorderSynchronization(boat_id=recorder_id,
                                   status='ok',
                                   timestamp=now.timestamp(),
                                   direct_message="Test Message",
                                   alerts=body.alerts,
                                   boat_route=body.boat_route)

    response = JSONResponse(content=data.model_dump(),
                            status_code=status.HTTP_200_OK)

    return response


@recorder_router.post("/{recorder_id}/position_update", response_model=PositionUpdate)
def recorder_position_update(recorder_id: int, body: PositionUpdate):
    from datetime import datetime

    ts_float = body.boat_position.timestamp
    ts_dt = datetime.utcfromtimestamp(ts_float)

    with get_db_cursor() as cur:
        cur.execute(
            """
            INSERT INTO dbo.YachtPosition (yacht_id, timestamp, latitude, longitude)
            VALUES (?, ?, ?, ?)
            """,
            (
                recorder_id,
                ts_dt,
                float(body.boat_position.latitude),
                float(body.boat_position.longitude),
            ),
        )

    data = PositionUpdate(
        boat_id=recorder_id,
        timestamp=ts_float,
        boat_position=body.boat_position,
        status="ok",
    )

    return JSONResponse(content=data.model_dump(),
                        status_code=status.HTTP_200_OK)
