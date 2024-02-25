import os
os.environ['MAVLINK20'] = '1' # for using MAVLink v2
from pymavlink import mavutil
import time
from crypto_funcitons import *

class Drone:
    def __init__(self, connection_string):
        self.connection = mavutil.mavlink_connection(connection_string)
        self.secret_key = b'12345678912345678912345678912345'  
        self.connection.setup_signing(self.secret_key, sign_outgoing=True, allow_unsigned_callback=None, initial_timestamp=None, link_id=5)
        

    def send_heartbeat(self):
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,0,0)
        print(f"Heartbeat sent")
        
    def send_spefic(self):
        ByteNum = 237 # 16 byte tag (253-16)
        payload = bytes(range(ByteNum)) # Random Bytes for data
        REPEAT = 100 # Number of times we send MAVLink packets to GCS
        counter = 0
        logging.info(time.time())
        while True:
            encrypted_payload = payload
            self.connection.mav.specific_usage_send(encrypted_payload)
            counter += 1
            if counter == REPEAT:
                break  

def main():   
    drone = Drone('tcp:127.0.0.1:14550')
    time.sleep(1)
    drone.send_heartbeat()
    time.sleep(1)
    drone.send_heartbeat()
    time.sleep(1) 
    drone.send_spefic()
    
if __name__ == "__main__":
    main()
