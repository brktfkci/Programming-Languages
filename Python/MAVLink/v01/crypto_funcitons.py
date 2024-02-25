from cryptography.hazmat.primitives.ciphers.aead import ChaCha20Poly1305
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives.ciphers.aead import AESGCMSIV
from cryptography.hazmat.primitives.ciphers.aead import AESOCB3
from cryptography.hazmat.primitives.ciphers.aead import AESCCM


def encrypt_ChaCha20Poly1305(data, aad, key, nonce):
    chacha = ChaCha20Poly1305(key)
    ciphertext = chacha.encrypt(nonce, data, aad)
    return ciphertext
    
def decrypt_ChaCha20Poly1305(data, aad, key, nonce):
    chacha = ChaCha20Poly1305(key)
    plaintext = chacha.decrypt(nonce, data, aad)
    return plaintext
    
def encrypt_AESGCM(data, aad, key, nonce):
    aesgcm = AESGCM(key)
    ciphertext = aesgcm.encrypt(nonce, data, aad)
    return ciphertext
    
def decrypt_AESGCM(data, aad, key, nonce):
    aesgcm = AESGCM(key)
    plaintext = aesgcm.decrypt(nonce, data, aad)
    return plaintext
    
def encrypt_AESGCMSIV(data, aad, key, nonce):
    aesgcmsiv = AESGCMSIV(key)
    ciphertext = aesgcmsiv.encrypt(nonce, data, aad)
    return ciphertext
    
def decrypt_AESGCMSIV(data, aad, key, nonce):
    aesgcmsiv = AESGCMSIV(key)
    plaintext = aesgcmsiv.decrypt(nonce, data, aad)
    return plaintext
    
def encrypt_AESOCB3(data, aad, key, nonce):
    aesocb = AESOCB3(key)
    ciphertext = aesocb.encrypt(nonce, data, aad)
    return ciphertext
    
def decrypt_AESOCB3(data, aad, key, nonce):
    aesocb = AESOCB3(key)
    plaintext = aesocb.decrypt(nonce, data, aad)
    return plaintext
    
def encrypt_AESCCM(data, aad, key, nonce):
    aesccm = AESCCM(key)
    ciphertext = aesccm.encrypt(nonce, data, aad)
    return ciphertext
    
def decrypt_AESCCM(data, aad, key, nonce):
    aesccm = AESCCM(key)
    plaintext = aesccm.decrypt(nonce, data, aad)
    return plaintext