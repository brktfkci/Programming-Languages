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


# Pre shared key and nonce 
key = b'12345678912345678912345678912345'  
nonce = b'123456789123'

class GCS:
    def __init__(self, connection_string):
        self.connection = mavutil.mavlink_connection(connection_string)     
        self.connection.wait_heartbeat()
        
    def receive_messages(self):
        REPEAT = 100
        counter = 0
        while True:
            msg = self.connection.recv_msg()
            if msg:
                if msg.msgname == "SPECIFIC_USAGE":
                    ####################################################################################################################################
                    # Decrypt the payload using one of them or none of them (Uncomment it)
                    ####################################################################################################################################
                    aad_unpacked = msg.get_header()
                    aad = aad_unpacked.pack()
                    msg_decrypted = msg._text_raw
                    # msg_decrypted = decrypt_ChaCha20Poly1305(msg._text_raw, aad, key, nonce)
                    # msg_decrypted = decrypt_AESGCM(msg._text_raw, aad, key, nonce)
                    # msg_decrypted = decrypt_AESGCMSIV(msg._text_raw, aad, key, nonce)
                    # msg_decrypted = decrypt_AESOCB3(msg._text_raw, aad, key, nonce)
                    # msg_decrypted = decrypt_AESCCM(msg._text_raw, aad, key, nonce)
                    ####################################################################################################################################
                    # print(msg_decrypted)
                    counter += 1
                    if counter == REPEAT:
                        # logging.info(time.time())
                        print("Finished")
                        break  
                else:
                    pass
            
if __name__ == "__main__":
    gcs = GCS('tcpin:127.0.0.1:14550') 
    time.sleep(1)
    gcs.receive_messages()
