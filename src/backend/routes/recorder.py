from datetime import datetime
from fastapi import APIRouter, status
from fastapi.responses import JSONResponse

from ..models.registration import Registration
from ..models.recorder_synchronization import RecorderSynchronization
from ..models.position_update import PositionUpdate

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
                                   direct_message="Test Message")

    response = JSONResponse(content=data.model_dump(),
                            status_code=status.HTTP_200_OK)

    return response


@recorder_router.post("/{recorder_id}/position_update", response_model=PositionUpdate)
def recorder_position_update(recorder_id: int, body: PositionUpdate):
    now = datetime.now()

    data = PositionUpdate(boat_id=recorder_id,
                          timestamp=now.timestamp(),
                          boat_position=body.boat_position)

    response = JSONResponse(content=data.model_dump(),
                            status_code=status.HTTP_200_OK)

    return response
