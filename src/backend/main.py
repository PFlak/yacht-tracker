from fastapi import FastAPI
from .routes.recorder import recorder_router
from .routes.ui import ui_router
from .routes.user import user_router

app = FastAPI()
app.include_router(recorder_router, prefix='/recorder')
app.include_router(ui_router, prefix='/ui')
app.include_router(user_router, prefix='/api/user')


@app.get("/")
def ping():
    return {"status": "ok"}
