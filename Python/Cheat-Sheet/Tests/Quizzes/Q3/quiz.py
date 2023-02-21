# -*- coding: utf-8 -*-
"""
Created on Mon Nov  2 18:53:30 2020

@author: Burak Tufekci S017309
"""

def rot13(msg:str, prcsType:int):
    myKey = {'a':'n','b':'o','c':'p','d':'q','e':'r','f':'s',
    'g':'t','h':'u','i':'v','j':'w','k':'x','l':'y',
    'm':'z','n':'a','o':'b','p':'c','q':'d','r':'e',
    's':'f','t':'g','u':'h','v':'i','w':'j','x':'k',
    'y':'l','z':'m','A':'N','B':'O','C':'P','D':'Q',
    'E':'R','F':'S','G':'T','H':'U','I':'V','J':'W',
    'K':'X','L':'Y','M':'Z','N':'A','O':'B','P':'C',
    'Q':'D','R':'E','S':'F','T':'G','U':'H','V':'I',
    'W':'J','X':'K','Y':'L','Z':'M'
    }
    cnvrted_msg = ""
    for ii in msg:
        #if myKey include incoming str change with matched key's value
        if ii in myKey:
            cnvrted_msg += myKey[ii]
        #if myKey does not include incoming str in its key
        else:
            cnvrted_msg += ii
    return cnvrted_msg

while True:
    m = input("Enter a message: ")
    tt = int(input("Encode= 1, Decode= 2. Which one? : "))
    #send m and tt to rot13 function
    message_str = rot13(m, tt)
    print("The message '{}' is {} as '{}'\n".format(m, "encryped" if tt == 1 else "decrypted", message_str))
    #append message_str in a proper way to secret.txt
    f = open("secret.txt", mode = 'a+', encoding = 'utf-8')
    f.write("The message '{}' is {} as '{}'\n".format(m, "encryped" if tt == 1 else "decrypted", message_str))
    f.close()
    #control loop to exit infinite loop
    ct_loop = input("Do you want to continue? If yes type 'Y' and press enter, otherwise type anything and press enter :")
    if ct_loop != "Y" or "y":
        break
