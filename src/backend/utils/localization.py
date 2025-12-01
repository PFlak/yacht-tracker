import random
from ..models.boat_position import BoatPosition
from .time import generate_random_timestamp


def generate_random_position() -> tuple[float, float]:
    return (float(random.random(480, 510) / 10), float(random.random(280, 300) / 10))


def generate_random_boat_position() -> BoatPosition:
    lt, lg = generate_random_position()
    bp = BoatPosition(id=random.randint(1, 100),
                      recorder_id=1,
                      timestamp=generate_random_timestamp(),
                      latitude=lt,
                      longitude=lg)

    return bp
