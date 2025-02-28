from optoforce import OptoForce22 as OptoForce
from optoforce.status import no_errors

with OptoForce(speed_hz=1000, filter_hz=150, zero=True) as force_sensor:
    while True:
        print(force_sensor.read(only_latest_data=False))
        
    
