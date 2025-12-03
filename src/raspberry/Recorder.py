import requests as re
from datetime import datetime
from models.boat_position import BoatPosition
from models.boat_route import BoatRoute
from models.position_update import PositionUpdate
from models.recorder_synchronization import RecorderSynchronization

from AdjustableTimer import AdjustableTimer

class Recorder:

    def __init__(self):
        self.id = 0
        self.default_sync_time = 5.0
        self.default_position_snapshot_time = 5.0
        self.update_time = 0.0

        self.server_url = 'http://srv28.mikr.us:40161'
        self.api_sync_path = f'{self.server_url}/recorder/{self.id}/sync'
        self.api_update_path = f'{self.server_url}/recorder/{self.id}/position_update'

        self.longitude = 0.0
        self.latitude = 0.0

        self.boat_route: BoatRoute = BoatRoute(boat_id=self.id)

        self.sync_timer = AdjustableTimer(period_s=self.default_sync_time,
                                          callback=self.cb_sync_timer)
        self.sync_timer.start()
        
        self.position_snapshot_timer = AdjustableTimer(period_s=self.default_position_snapshot_time,
                                                       callback=self.cb_position_snapshot_timer)
        
        self.position_snapshot_timer.start()

        self.update_timer = None

        while True:
            ...

    def __del__(self):
        self.sync_timer.stop()

    def cb_sync_timer(self):
        print(f"Sync Time: {datetime.now().strftime('%d/%m/%Y, %H:%M:%S')}")
        now = datetime.now().timestamp()

        recorder_synchronization = RecorderSynchronization(boat_id=self.id,
                                                           status='ok',
                                                           timestamp=now,
                                                           sync_period=self.default_sync_time,
                                                           position_update_period=self.update_time,
                                                           position_stamp_period=self.default_position_snapshot_time,
                                                           boat_route=self.boat_route if len(self.boat_route.positions) else None)

        body = recorder_synchronization.model_dump_json()

        try:
            response = re.post(self.api_sync_path, data=body)
            if response.status_code != 200:
                raise Exception()

            self.boat_route = BoatRoute(boat_id=self.id)

            data: dict = response.json()
            print(f"Sync update: {data}")

            data: RecorderSynchronization = RecorderSynchronization.model_validate(data)

            if data.sync_period != self.default_sync_time:
                self.default_sync_time = data.sync_period
                self.sync_timer.set_period(self.default_sync_time)

            if data.position_update_period != self.update_time:
                self.update_time = data.position_update_period

                if not self.update_timer:
                    self.update_timer = AdjustableTimer(period_s=self.update_time,
                                                        callback=self.cb_update_timer)

                    self.update_timer.start()

                self.update_timer.set_period(self.default_sync_time)

            if data.position_stamp_period != self.default_position_snapshot_time:
                self.default_position_snapshot_time = data.position_stamp_period
                self.position_snapshot_timer.set_period(self.default_position_snapshot_time)

            self.display_text_message(data.direct_message)

        except Exception as e:
            print(f"Error: {e}")

    def cb_position_snapshot_timer(self):
        print(f"Position Snapshot Time: {datetime.now().strftime('%d/%m/%Y, %H:%M:%S')}")
        now = datetime.now().timestamp()

        boat_position = BoatPosition(id=0,
                                     recorder_id=self.id,
                                     timestamp=now,
                                     latitude=self.latitude,
                                     longitude=self.longitude)

        self.boat_route.positions.append(boat_position)

    def cb_update_timer(self):
        print(f"Update Time: {datetime.now().strftime('%d/%m/%Y, %H:%M:%S')}")
        try:
            now = datetime.now().timestamp()

            boat_position = BoatPosition(id=0,
                                         recorder_id=self.id,
                                         timestamp=now,
                                         latitude=self.latitude,
                                         longitude=self.longitude)

            position_update = PositionUpdate(boat_id=self.id,
                                             timestamp=now,
                                             boat_position=boat_position,
                                             status='ok')

            body = position_update.model_dump_json()

            response = re.post(self.api_update_path, data=body)

        except Exception as e:
            print(f"Error: {e}")

    def display_text_message(self, msg: str):
        print(f"Displaying: {msg}")