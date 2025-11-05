from fastapi.routing import APIRouter
from ..models.boats import NewBoat, Boat

recorder_router = APIRouter()


@recorder_router.post('/', response_model=Boat)
def create_recorder(body: NewBoat):
    pass


@recorder_router.put('/{recorder_id}', response_model=Boat)
def update_recorder(recorder_id: int):
    pass


@recorder_router.get('/alerts')
def get_recorders_incidents(last_h: int = 24):
    pass


@recorder_router.get('/{recorder_id}/alerts')
def get_recorder_incidents(recorder_id: int, last_h: int = 24, limit: int = 10):
    pass


@recorder_router.get('/routes')
def get_recorders_routes(history: bool = False, last: int = 0):
    pass


@recorder_router.get('{recorder_id}/route')
def get_recorder_route(recorder_id: int, history: bool = True, depth: int = 100):
    pass
