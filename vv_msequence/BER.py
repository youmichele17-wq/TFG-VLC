def calc_BER(rx_bits, tx_bits, L=None):

    if L is None:
        L = min(len(rx_bits), len(tx_bits))
    else:
        L = min(L, len(rx_bits), len(tx_bits))

    if L <= 0:
        return 1.0

    errors = 0

    for i in range(L):
        r = rx_bits[i]
        t = tx_bits[i]

        if r not in (0, 1):
            errors += 1
        elif r != t:
            errors += 1

    return errors / L