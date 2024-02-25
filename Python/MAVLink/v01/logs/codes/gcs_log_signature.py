import os
os.environ['MAVLINK20'] = '1' # for using MAVLink v2
from pymavlink import mavutil
import time
from crypto_funcitons import *
import logging

###################
# Uncommment it if you want to measure time difference
logging.basicConfig(filename='timestamp_log_gcs.log', level=logging.INFO, format='%(message)s')
###################

class GCS:
    def __init__(self, connection_string):
        self.connection = mavutil.mavlink_connection(connection_string)     
        self.secret_key = b'12345678912345678912345678912345'  
        self.connection.setup_signing(self.secret_key, sign_outgoing=True, allow_unsigned_callback=None, initial_timestamp=None, link_id=6)        
        self.connection.wait_heartbeat()
        
    def receive_messages(self):
        REPEAT = 100
        counter = 0
        while True:
            msg = self.connection.recv_msg()
            if msg:
                if msg.msgname == "SPECIFIC_USAGE":
                    msg_decrypted = msg._text_raw
                    counter += 1
                    if counter == REPEAT:
                        logging.info(time.time())
                        print("Finished")
                        break  
                else:
                    pass
            
if __name__ == "__main__":
    gcs = GCS('tcpin:127.0.0.1:14550') 
    time.sleep(1)
    gcs.receive_messages()
