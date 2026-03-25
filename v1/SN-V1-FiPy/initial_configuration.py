from network import LTE
import time
import socket

lte = LTE()
lte.attach()

print("attaching..",end='')
while not lte.isattached():
    print('#',end='')
    time.sleep(0.5)

lte.connect(cid = 3)
print("connecting [##",end='')
while not lte.isconnected():
    time.sleep(0.25)
    print('#',end='')
    print(lte.send_at_cmd('AT!="fsm"'))
print("] connected!")

print(socket.getaddrinfo('pybytes.pycom.io', 80))  
lte.deinit()