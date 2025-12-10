import sqlite3
from typing import Optional, Dict, Any

from shapely.geometry import Point, Polygon
from geopy.distance import geodesic


class CollisionChecker:

    def __init__(
        self,
        db_path: str,
        alert_radius_m: float = 50.0,
        debug_radius_m: float = 2000.0,
    ) -> None:
        self.db_path = db_path
        self.alert_radius_m = alert_radius_m
        self.debug_radius_m = debug_radius_m

        self.hazards = []
        self.buoys = []

        self._load_data()

    def _load_data(self) -> None:
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()

        cur.execute(
            """
            SELECT LakeName, AreaId, Name, HazardType, GeomWKT
            FROM HazardAreasMerged
            """
        )
        for lake, area_id, name, hazard_type, wkt in cur.fetchall():
            poly = self._polygon_from_wkt(wkt)
            self.hazards.append(
                {
                    "LakeName": lake,
                    "AreaId": area_id,
                    "Name": name,
                    "HazardType": hazard_type,
                    "polygon": poly,
                }
            )

        cur.execute(
            """
            SELECT LakeName, BuoyId, seamark_type, category, Latitude, Longitude
            FROM BuoysMerged
            """
        )
        for lake, buoy_id, seamark_type, category, lat, lon in cur.fetchall():
            self.buoys.append(
                {
                    "LakeName": lake,
                    "BuoyId": buoy_id,
                    "seamark_type": str(seamark_type).strip() if seamark_type else "",
                    "category": str(category).strip() if category else "",
                    "lat": float(lat),
                    "lon": float(lon),
                }
            )

        conn.close()

    def _polygon_from_wkt(self, wkt: str) -> Polygon:
        wkt = wkt.strip()
        wkt = wkt.replace("POLYGON ((", "").replace("POLYGON(", "")
        wkt = wkt.replace("))", "").replace(")", "")

        coords = []
        for c in wkt.split(","):
            c = c.strip()
            if not c:
                continue
            lon_str, lat_str = c.split()
            lon = float(lon_str)
            lat = float(lat_str)
            coords.append((lon, lat))

        return Polygon(coords)

    def check_collision(self, lat: float, lon: float) -> Dict[str, Any]:
        boat_point = Point(lon, lat)

        hazard_result: Dict[str, Any] = {"inside": False, "hazard": None}
        for hz in self.hazards:
            if hz["polygon"].contains(boat_point):
                hazard_result["inside"] = True
                hazard_result["hazard"] = {
                    "LakeName": hz["LakeName"],
                    "AreaId": hz["AreaId"],
                    "Name": hz["Name"],
                    "HazardType": hz["HazardType"],
                }
                break

        buoy_result: Dict[str, Any] = {"alert": False, "distance_m": None, "buoy": None}

        closest_buoy = None
        closest_dist = None

        for b in self.buoys:
            buoy_pos = (b["lat"], b["lon"])
            boat_pos = (lat, lon)
            dist_m = geodesic(boat_pos, buoy_pos).meters

            if closest_dist is None or dist_m < closest_dist:
                closest_dist = dist_m
                closest_buoy = b

        if closest_buoy is not None:
            buoy_result["distance_m"] = closest_dist
            buoy_result["buoy"] = closest_buoy
            if closest_dist <= self.alert_radius_m:
                buoy_result["alert"] = True

        return {
            "hazard": hazard_result,
            "buoy": buoy_result,
        }

    def build_alert_message(self, collision: Dict[str, Any]) -> Optional[str]:
        if collision["hazard"]["inside"] and collision["hazard"]["hazard"]:
            hz = collision["hazard"]["hazard"]
            return (
                f"UWAGA! Wpływasz do strefy niebezpiecznej: "
                f"{hz['Name']} (typ: {hz['HazardType']}, jezioro: {hz['LakeName']})"
            )

        if collision["buoy"]["alert"] and collision["buoy"]["buoy"]:
            b = collision["buoy"]["buoy"]
            dist = int(collision["buoy"]["distance_m"] or 0)
            return (
                f"Uwaga! Zbliżasz się do boi {b['category']} (ID={b['BuoyId']}) "
                f"– dystans {dist} m"
            )

        return None
