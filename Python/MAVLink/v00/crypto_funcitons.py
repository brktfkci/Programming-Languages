from Crypto.Cipher import AES
from Crypto.Cipher import ChaCha20
from Crypto.Util import Counter
from Crypto.Random import get_random_bytes
from base64 import b64encode, b64decode
        
def encrypt_xor(plaintext, key):
    key = key * ((len(plaintext) // len(key)) + 1)
    ciphertext = bytes([a ^ b for a, b in zip(plaintext, key)])
    return ciphertext

def decrypt_xor(ciphertext, key):
    key = key * ((len(ciphertext) // len(key)) + 1)
    plaintext = bytes([a ^ b for a, b in zip(ciphertext, key)])
    return plaintext
    
def encrypt_aes_ctr(plaintext, key):
    cipher = AES.new(key, AES.MODE_CTR, counter=Counter.new(128))
    ciphertext = cipher.encrypt(plaintext)
    return ciphertext

def decrypt_aes_ctr(ciphertext, key):
    cipher = AES.new(key, AES.MODE_CTR, counter=Counter.new(128))
    plaintext = cipher.decrypt(ciphertext)
    return plaintext    
    
def encrypt_aes_gcm(plaintext, key):
    iv = get_random_bytes(16)
    cipher = AES.new(key, AES.MODE_GCM, nonce=iv)
    ciphertext, tag = cipher.encrypt_and_digest(plaintext)
    encrypted_data = iv + ciphertext + tag
    return encrypted_data
                                                                                                
def decrypt_aes_gcm(encrypted_data, key):
    iv = encrypted_data[:16]
    ciphertext = encrypted_data[16:-16]
    tag = encrypted_data[-16:]
    cipher = AES.new(key, AES.MODE_GCM, nonce=iv)
    plaintext = cipher.decrypt_and_verify(ciphertext, tag)
    return plaintext
    
def encrypt_aes_gcm_assoc(plaintext, key, associated_data):
    iv = get_random_bytes(16)
    cipher = AES.new(key, AES.MODE_GCM, nonce=iv)
    cipher.update(associated_data) 
    ciphertext, tag = cipher.encrypt_and_digest(plaintext)
    encrypted_data = iv + ciphertext + tag
    return encrypted_data

def decrypt_aes_gcm_assoc(encrypted_data, key, associated_data):
    iv = encrypted_data[:16]
    ciphertext = encrypted_data[16:-16]
    tag = encrypted_data[-16:]
    cipher = AES.new(key, AES.MODE_GCM, nonce=iv)
    cipher.update(associated_data)  
    plaintext = cipher.decrypt_and_verify(ciphertext, tag)
    return plaintext

def encrypt_chacha20(plaintext, key, nonce):
    cipher = ChaCha20.new(key=key, nonce=nonce)
    ciphertext = cipher.encrypt(plaintext)
    return ciphertext

def decrypt_chacha20(ciphertext, key, nonce):
    cipher = ChaCha20.new(key=key, nonce=nonce)
    plaintext = cipher.decrypt(ciphertext)
    return plaintext