import pyodbc
from contextlib import contextmanager

SERVER = "database-1.c5cwy4umqi0k.eu-north-1.rds.amazonaws.com"
DATABASE = "FleetMonitoring"
USERNAME = "admin"
PASSWORD = "Fleet1234!"
DRIVER = "{ODBC Driver 18 for SQL Server}"

CONN_STR = (
    f"DRIVER={DRIVER};"
    f"SERVER={SERVER};"
    f"DATABASE={DATABASE};"
    f"UID={USERNAME};"
    f"PWD={PASSWORD};"
    "TrustServerCertificate=yes;"
)


@contextmanager
def get_db_cursor():
    conn = pyodbc.connect(CONN_STR)
    try:
        cursor = conn.cursor()
        yield cursor
        conn.commit()
    finally:
        cursor.close()
        conn.close()
