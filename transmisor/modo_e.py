import serial, time, sys
from reedsolo import RSCodec

PORT = "/dev/ttyAMA0"
BAUD = 38400
INTER_ROWS = 15

MCS_MODO_E = 4

FLP = [1, 0] * 32
TDP = [0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0]
TDP_INV = [1 - b for b in TDP]
SHR_BITS = FLP + TDP + TDP_INV + TDP + TDP_INV

RS_K_HEADER = 7
HEADER_RATE = "1/4"

G = [0o133, 0o171, 0o165]
K = 7
MASK = (1 << K) - 1

def make_rs(k):
    return RSCodec(nsym=15-k, nsize=15, c_exp=4, prim=0x13, fcr=1, generator=2)

RS_7  = make_rs(7)
RS_11 = make_rs(11)

def get_rs(k):
    return RS_7 if k == 7 else RS_11

def parity(x: int) -> int:
    p = 0
    while x:
        p ^= 1
        x &= x - 1
    return p

def bytes_to_bits(data: bytes):
    return [((b >> i) & 1) for b in data for i in range(8)]

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

def pad_to_multiple(bits, m, pad=0):
    r = len(bits) % m
    return list(bits) if r == 0 else list(bits) + [pad] * (m - r)

def crc16_hcs(bits, init=0xFFFF):
    crc = init
    poly = 0x8408
    for bit in bits:
        fb = (crc ^ (bit & 1)) & 1
        crc >>= 1
        if fb:
            crc ^= poly
    return crc & 0xFFFF

def u16_to_bits(x: int):
    return [(x >> i) & 1 for i in range(16)]

def build_phr_bits(payload_len_bytes: int, payload_mcs_id: int):
    phr = [0, 0, 0, 0]
    phr.extend([(payload_mcs_id >> i) & 1 for i in range(6)])
    phr.extend([(payload_len_bytes >> i) & 1 for i in range(16)])
    phr.extend([0] * 6)
    return phr

def mseq():
    state = [1, 0, 0, 0, 0, 0, 0]
    seq = []
    L = 2**7 - 1
    for _ in range(L):
        seq.append(state[-1])
        new = state[0] ^ state[-1]
        state = [new] + state[:-1]
    return seq

def bytes_to_nibbles(data: bytes):
    out = []
    for b in data:
        out.append((b >> 4) & 0xF)
        out.append(b & 0xF)
    return out

def pack_nibbles_to_bytes(nibs):
    out = bytearray()
    i = 0
    while i < len(nibs):
        hi = nibs[i] & 0xF
        lo = (nibs[i+1] & 0xF) if (i+1) < len(nibs) else 0
        out.append((hi << 4) | lo)
        i += 2
    return bytes(out)

def rs16_encode(payload_bytes: bytes, rs_k: int) -> bytes:
    rs = get_rs(rs_k)
    data_syms = bytes_to_nibbles(payload_bytes)
    coded_syms = []
    i = 0
    while i < len(data_syms):
        blk = data_syms[i:i+rs_k]
        s = len(blk)
        if s == rs_k:
            enc = rs.encode(bytes(blk))
            coded_syms.extend(enc)
        else:
            pre = rs_k - s
            msg = bytes(([0] * pre) + blk)
            enc = rs.encode(msg)
            coded_syms.extend(enc[pre:])
        i += rs_k
    return pack_nibbles_to_bytes(coded_syms)

def interleave_and_puncture(bits, rows=15):
    N = len(bits)
    if N == 0:
        return []
    cols = (N + rows - 1) // rows
    pad_len = (rows * cols) - N

    b_padded = list(bits) + [0] * pad_len
    mask_padded = [1] * N + [0] * pad_len

    matrix_data = [b_padded[r*cols:(r+1)*cols] for r in range(rows)]
    matrix_mask = [mask_padded[r*cols:(r+1)*cols] for r in range(rows)]

    out_bits = []
    for c in range(cols):
        for r in range(rows):
            if matrix_mask[r][c] == 1:
                out_bits.append(matrix_data[r][c])
    return out_bits

def cc_encode_1_3(info_bits):
    state = 0
    out = []
    for b in list(info_bits) + [0] * (K - 1):
        state = ((state << 1) | (b & 1)) & MASK
        out.append(parity(state & G[0]))
        out.append(parity(state & G[1]))
        out.append(parity(state & G[2]))
    return out

def rate_map_1_4(mother_bits):
    out = []
    for i in range(0, len(mother_bits), 6):
        if i + 5 < len(mother_bits):
            half = [mother_bits[i], mother_bits[i+1],
                    mother_bits[i+3], mother_bits[i+4]]
            for b in half:
                out.extend([b, b])
    return out

def process_chain_header(bits):
    b_rs = rs16_encode(bits_to_bytes(bits), RS_K_HEADER)
    b_rs_bits = bytes_to_bits(b_rs)
    b_int = interleave_and_puncture(b_rs_bits, INTER_ROWS)
    b_cc = cc_encode_1_3(b_int)
    b_cc = pad_to_multiple(b_cc, 6, 0)
    return rate_map_1_4(b_cc)

def manchester(bits):
    out = []
    for b in bits:
        out.extend([0, 1] if b == 1 else [1, 0])
    return out

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1, write_timeout=1)
    except Exception as e:
        print("Error puerto:", e)
        sys.exit(1)

    print(f"TX - Modo E (MCS ID = {MCS_MODO_E}, sin RS, sin CC)")
    n = 0

    try:
        for _ in range(200):
            n += 1
            print(n)

            payload_raw = mseq()
            payload_len_bytes = len(bits_to_bytes(payload_raw))
            payload_bits_len = payload_len_bytes * 8
            payload = payload_raw + [0] * (payload_bits_len - len(payload_raw))

            phr_bits = build_phr_bits(payload_len_bytes, MCS_MODO_E)
            hcs = crc16_hcs(phr_bits)
            header_bits = phr_bits + u16_to_bits(hcs)

            header_coded = process_chain_header(header_bits)

            psdu = SHR_BITS + header_coded + payload
            data_bits = manchester(psdu)
            data_bytes = bits_to_bytes(data_bits)

            ser.write(data_bytes)
            ser.flush()
            time.sleep(0.05)

        time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nTX terminado.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()