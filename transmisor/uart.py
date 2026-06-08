#UART TX

import serial
import time

ser = serial.Serial('/dev/ttyAMA0', 38400, timeout=1)

def bits_to_bytes(bits):
    bits = list(bits)
    while len(bits) % 8 != 0:
        bits.append(0)
    out = bytearray()
    for i in range(0, len(bits), 8):
        v = 0
        for j in range(8):
            v |= (bits[i + j] & 1) << j
        out.append(v)
    return bytes(out)

n = 0

for i in range(200):
    state = [1,0,0,0,0,0,0]
    out = state[-1]
    seq = []

    L = 2**7 - 1

    for i in range(L):
        out = state[-1]
        seq.append(out)

        new = state[0] ^ state[-1]
        state = [new] + state[:-1]

    msg = bits_to_bytes(seq)

#    input()

    ser.write(msg)

    n += 1
    print(n)

    ser.flush()
    time.sleep(0.05)
