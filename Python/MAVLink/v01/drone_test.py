import os
os.environ['MAVLINK20'] = '1' # for using MAVLink v2
from pymavlink import mavutil
import time
from crypto_funcitons import *


# Pre shared key and nonce 
key = b'12345678912345678912345678912345'  
nonce = b'123456789123'

class Drone:
    def __init__(self, connection_string):
        self.connection = mavutil.mavlink_connection(connection_string)
        self.counter = 0

    def send_heartbeat(self):
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,0,0)
        print(f"Heartbeat sent")
        
    def send_with_header(self, msg: mavutil.mavlink.MAVLink_message, force_mavlink1: bool = False) -> None:
        """
        Wrapper around the existing send function to print header information before sending.

        msg                : MAVLink message object.
        force_mavlink1     : Force the use of MAVLink v1.

        """

        msg_header_ptr = msg.get_header()
        msg_header_ptr.seq = self.connection.mav.seq
        msg_header_ptr.srcSystem = self.connection.mav.srcSystem
        msg_header_ptr.srcComponent = self.connection.mav.srcComponent
        # msg_header_ptr.mlen = len(msg._text_raw) + 16 # tag size of encrypted data = 16 # uncomment it if you want to try encryption algorithms, comment the below line.
        msg_header_ptr.mlen = len(msg._text_raw) 
        aad = msg_header_ptr.pack()
        data = msg._text_raw
        
        ####################################################################################################################################
        # Encrypt the payload using one of them or none of them (Uncomment it)
        ####################################################################################################################################
        encrypted_payload = data
        # encrypted_payload = encrypt_ChaCha20Poly1305(data, aad, key, nonce)
        # encrypted_payload = encrypt_AESGCM(data, aad, key, nonce)
        # encrypted_payload = encrypt_AESGCMSIV(data, aad, key, nonce)
        # encrypted_payload = encrypt_AESOCB3(data, aad, key, nonce)
        # encrypted_payload = encrypt_AESCCM(data, aad, key, nonce)
        ####################################################################################################################################    
        msg._text_raw = encrypted_payload

        # Call the existing send function
        self.connection.mav.send(msg, force_mavlink1=force_mavlink1)
                
    def send_spefic(self):
        ByteNum = 237 # 16 byte tag (253-16)
        payloadByteNum = bytes(range(ByteNum)) # Random Bytes for data
        REPEAT = 100 # Number of times we send MAVLink packets to GCS

        # logging.info(time.time())
        while True:
            specific_usage_msg = self.connection.mav.specific_usage_encode(payloadByteNum)
            self.send_with_header(specific_usage_msg, force_mavlink1=False)
            self.counter += 1
            if self.counter == REPEAT:
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


