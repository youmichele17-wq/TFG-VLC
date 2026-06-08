def mseq():
    state = [1,0,0,0,0,0,0]
    seq = []
    L = 2**7 - 1

    for _ in range(L):
        seq.append(state[-1])
        new = state[0] ^ state[-1]
        state = [new] + state[:-1]

    return seq


def calc_BER(rx_bits):
    tx_bits = mseq()
    rx_bits = rx_bits[:len(tx_bits)]

    errors = 0

    for i in range(len(tx_bits)):
        if rx_bits[i] not in (0,1):
            errors += 1
        elif tx_bits[i] != rx_bits[i]:
            errors += 1

    return errors / len(tx_bits) 
