from fastapi.routing import APIRouter
from .ui_recorder import recorder_router

ui_router = APIRouter()

ui_router.include_router(recorder_router, prefix='/recorder')
