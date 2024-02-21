#!/usr/bin/env python3

from crypto_funcitons import *
from ascon._ascon import ascon_encrypt 
from ascon._ascon import ascon_decrypt
import time
import os
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil

# Define your shared key 
shared_key_cha = b'12345678912345678912345678912345'  
shared_nonce_cha = b'12345678'
shared_key_ascon = b'1234567891234567'  
shared_nonce_ascon = b'1234567891234567'
shared_aod_ascon = b'1234567'
shared_key_xor = b'12345678912345678912345678912345'  
shared_key_aes = b'1234567891234567'   
shared_aod_aes = b'1234567'   
shared_nonce_aes = b'1234567891234567'

class Drone:
    def __init__(self, connection_string):
        self.connection = mavutil.mavlink_connection(connection_string)
        self.secret_key = bytearray(chr(42)*32, 'utf-8' )  
        self.connection.setup_signing(self.secret_key, sign_outgoing=True, allow_unsigned_callback=None, initial_timestamp=None, link_id=None)
        
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
        TRIAL = 10
        for ii in range(0,TRIAL+1):
            counter = 0
            # start1 = time.perf_counter()
            print(time.time())
            while True:
                msg = self.connection.recv_msg()
                if msg:
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
                    counter += 1
                    if counter == REPEAT:
                        # finish1 = time.perf_counter()
                        # print(time.time())
                        break  

                else:
                    pass
            # total_time = ((finish1-start1)*1000)/REPEAT
            # print(f"Total time elapsed = {total_time}")
            # with open("output.txt", "a") as file:
                # file.write(str(total_time))
                # file.write("\n")

        # with open("output.txt", "a") as file:
            # file.write("-----------------")
            # file.write("\n")

if __name__ == "__main__":
    drone = Drone('tcp:127.0.0.1:14550')
    time.sleep(1)
    drone.send_heartbeat()
    time.sleep(1)
    drone.send_heartbeat()
    time.sleep(1)    
    drone.send_spefic()
