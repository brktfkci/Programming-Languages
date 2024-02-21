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

class Drone:
    def __init__(self, connection_string):
        self.connection = mavutil.mavlink_connection(connection_string)
        self.private_key = x25519.X25519PrivateKey.generate()
        self.public_key = self.private_key.public_key()
        self.secret_key = bytearray(chr(42)*32, 'utf-8' )  
        self.connection.setup_signing(self.secret_key, sign_outgoing=True, allow_unsigned_callback=None, initial_timestamp=None, link_id=5)
        
    def signing_enable(self):
        self.connection.mav.setup_signing_send(self.connection.target_system, self.connection.target_component, list(range(1, 33)), 123456789)    

    def sign_message(self, message):
        self.connection.sign_packet(message)
        
    def send_key(self):
        public_key_bytes = self.public_key.public_bytes(encoding=serialization.Encoding.Raw, format=serialization.PublicFormat.Raw)
        self.connection.mav.statustext_send(123, public_key_bytes) 
      
    def send_heartbeat(self):
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,0,0)
        print(f"Heartbeat sent")
        
    def send_spefic(self):
        ByteNum = 208 # AES GCM
        # ByteNum = 239 # ASCON
        # ByteNum = 255
        payload = bytes(range(ByteNum))
        REPEAT = 100
        counter = 0
        print(time.time())
        while True:
            ####################################################################################################################################
            # Encrypt the payload using one of them or none of them (Uncomment it)
            ####################################################################################################################################
            encrypted_payload = payload
            # encrypted_payload = encrypt_xor(payload, shared_key_xor)
            # encrypted_payload = encrypt_aes_gcm(payload, shared_key_aes)
            # encrypted_payload = encrypt_aes_gcm_assoc(payload, shared_key_aes, shared_aod_aes)
            # encrypted_payload = encrypt_aes_ctr(payload, shared_key_aes)
            # encrypted_payload = encrypt_chacha20(payload, shared_key_cha, shared_nonce_cha)
            # encrypted_payload = ascon_encrypt(shared_key_ascon, shared_nonce_ascon, shared_aod_ascon, payload, variant="Ascon-128a")
            ####################################################################################################################################
            self.connection.mav.specific_usage_send(encrypted_payload)
            # print(f"encrypted_payload = {encrypted_payload}")
            
            # time.sleep(1)
            counter += 1
            if counter == REPEAT:
                break  
    
    def receive_messages(self):
        while True:
            msg = self.connection.recv_msg()
            if msg:
                if msg.msgname == "STATUSTEXT":
                    public_key_gcs = x25519.X25519PublicKey.from_public_bytes(msg._text_raw)
                    shared_secret = self.private_key.exchange(public_key_gcs)
                    print(f"Shared key = {shared_secret}")
                    self.shared_key = shared_secret
                elif msg.msgname == "SPECIFIC_USAGE":
                    pass
                    # self.connection.mav.specific_usage_send()
                else:
                    pass
                    # print(f"Received message: {msg}")
                    # self.connection.mav.command_ack_send(1, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            time.sleep(1)

if __name__ == "__main__":
    drone = Drone('tcp:127.0.0.1:14550')
    # drone.signing_enable()
    time.sleep(1)
    drone.send_heartbeat()
    time.sleep(1)
    drone.send_key()
    time.sleep(1)
    drone.send_heartbeat()
    # time.sleep(1)    
    drone.send_spefic()
