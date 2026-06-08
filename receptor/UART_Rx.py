#UART RX

import serial
import time

ser = serial.Serial('/dev/ttyAMA0', 38400, timeout=1)

def mseq():

    state = [1,0,0,0,0,0,0]
    out = state[-1]
    seq = []
    L = 2**7 - 1

    for i in range(L):
        out = state[-1]
        seq.append(out)

        new = state[0] ^ state[-1]
        state = [new] + state[:-1]

    return seq

def bytes_to_bits(data: bytes):
    return [((b >> i) & 1) for b in data for i in range(8)]

def BER(rx_bits):
    tx_bits = mseq()
    rx_bits = rx_bits[:len(tx_bits)]

    errors = 0

    for i in range(len(tx_bits)):
        if tx_bits[i] != rx_bits[i]:
            errors += 1

    return errors/len(tx_bits)

def main():
    ber_total = 0
    i = 1
    tramas_correctas = 0
    ser.reset_input_buffer()

    while True:
        msg = ser.read(16)

        if len(msg) < 16:
            continue

        rx_bits = bytes_to_bits(msg)
        rx_bits = rx_bits[:127]

        ber = BER(rx_bits)
        print("Trama: ", i)
        tramas_correctas += 1
        print("Tramas correctas: ", tramas_correctas)
        print("BER:", ber)
        ber_total = ber + ber_total
        print(ber_total / tramas_correctas)

        i += 1

        #print(rx_bits)

        time.sleep(0.05)



if __name__ == "__main__":
    main()
