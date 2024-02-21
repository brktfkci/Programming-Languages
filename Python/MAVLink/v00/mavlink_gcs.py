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
        self.secret_key = bytearray(chr(42)*32, 'utf-8' )  
        self.connection.setup_signing(self.secret_key, sign_outgoing=True, allow_unsigned_callback=None, initial_timestamp=None, link_id=6)        
        self.connection.wait_heartbeat()
        
    def signing_enable(self):
        self.connection.mav.setup_signing_send(self.connection.target_system, self.connection.target_component, list(range(1, 33)), 123456789)    

    def sign_message(self, message):
        self.connection.sign_packet(message)

    def send_key(self):
        public_key_bytes = self.public_key.public_bytes(encoding=serialization.Encoding.Raw, format=serialization.PublicFormat.Raw)
        self.connection.mav.statustext_send(123, public_key_bytes)     
        
    def receive_messages(self):
        REPEAT = 100
        counter = 0
        while True:
            msg = self.connection.recv_msg()
            if msg:
                if msg.msgname == "STATUSTEXT":
                    public_key_drone = x25519.X25519PublicKey.from_public_bytes(msg._text_raw)
                    shared_secret = self.private_key.exchange(public_key_drone)
                    print(f"Shared key = {shared_secret}")
                    self.shared_key = shared_secret
                elif msg.msgname == "SPECIFIC_USAGE":
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
                    # self.connection.mav.command_ack_send(1, mavutil.mavlink.MAV_RESULT_ACCEPTED)
                    # if msg.get_signed():
                        # print(f"decrypted_payload = {msg_decrypted}")
                    counter += 1
                    if counter == REPEAT:
                        print(time.time())
                        break  
                    # time.sleep(1)
                else:
                    pass
                    # print(f"Received message: {msg}")
                    # self.connection.mav.command_ack_send(1, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            
if __name__ == "__main__":
    gcs = GCS('tcpin:127.0.0.1:14550') 
    # gcs.signing_enable()
    time.sleep(1)
    gcs.send_key()
    time.sleep(1)
    gcs.receive_messages()
