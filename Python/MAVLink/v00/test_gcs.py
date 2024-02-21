import os
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
import time
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives.asymmetric import x25519
from cryptography.hazmat.primitives import serialization
from ascon._ascon import ascon_encrypt 
from ascon._ascon import ascon_decrypt
from crypto_funcitons import *

# Define your shared key 
shared_key_cha = b'12345678912345678912345678912345'  
shared_nonce_cha = b'12345678'
shared_key_ascon = b'1234567891234567'  
shared_nonce_ascon = b'1234567891234567'
shared_aod_ascon = b'asdfgh'
shared_key_xor = b'12345678912345678912345678912345'  
shared_key_aes = b'1234567891234567'   
shared_aod_aes = b'12345678'
shared_nonce_aes = b'1234567891234567'  

class GCS:
    def __init__(self, connection_string):
        self.connection = mavutil.mavlink_connection(connection_string)
        self.private_key = x25519.X25519PrivateKey.generate()
        self.public_key = self.private_key.public_key()
        self.connection.wait_heartbeat()
        
    def signing_enable(self):
        self.connection.mav.setup_signing_send(self.connection.target_system, self.connection.target_component, list(range(1, 33)), 123456789)    

    def send_key(self):
        public_key_bytes = self.public_key.public_bytes(encoding=serialization.Encoding.Raw, format=serialization.PublicFormat.Raw)
        self.connection.mav.statustext_send(123, public_key_bytes)     
        
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
                    msg_decrypted = msg._text_raw
                    # msg_decrypted = decrypt_xor(msg._text_raw, shared_key_xor)
                    # msg_decrypted = decrypt_aes_gcm(msg._text_raw, shared_key_aes)
                    # msg_decrypted = decrypt_aes_gcm_assoc(msg._text_raw, shared_key_aes, shared_aod_aes)
                    # msg_decrypted = decrypt_aes_ctr(msg._text_raw, shared_key_aes)
                    # msg_decrypted = decrypt_chacha20(msg._text_raw, shared_key_cha, shared_nonce_cha)
                    # msg_decrypted = ascon_decrypt(shared_key_ascon, shared_nonce_ascon, shared_aod_ascon, msg._text_raw, variant="Ascon-128")
                    ####################################################################################################################################
                    self.connection.mav.command_ack_send(1, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                    counter += 1
                    if counter == REPEAT:
                        counter = 0
                        print(time.time())
                elif msg.msgname == "HEARTBEAT":
                    if hasattr(msg, 'SIGNING'):
                        print(f"Received message: {msg}")
                        self.connection.mav.command_ack_send(1, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                else:
                    pass
                    # print(f"Received message: {msg}")
                    # self.connection.mav.command_ack_send(1, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            
if __name__ == "__main__":
    gcs = GCS('tcpin:127.0.0.1:14550') 
    time.sleep(1)
    gcs.receive_messages()
