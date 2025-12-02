from datetime import datetime
from random import randint


def generate_random_timestamp() -> datetime:
    return datetime(year=randint(2020, 2025),
                    month=randint(4, 10),
                    hour=randint(6, 20),
                    minute=randint(0, 60),
                    day=randint(1, 30))


def now_as_float() -> float:
    return datetime.now().timestamp()
