from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from .routes.recorder import recorder_router
from .routes.ui import ui_router
import os

app = FastAPI()

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATIC_DIR = os.path.join(BASE_DIR, "frontend", "static")

app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

app.include_router(recorder_router, prefix='/recorder')
app.include_router(ui_router, prefix='/ui')


@app.get("/")
def ping():
    return {"status": "ok"}
