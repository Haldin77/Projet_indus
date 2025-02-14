from optoforce import OptoForce22 as OptoForce
from optoforce.status import no_errors

with OptoForce(speed_hz=10, filter_hz=15, zero=False) as force_sensor:
    while True:
        print(force_sensor.read(only_latest_data=False))
