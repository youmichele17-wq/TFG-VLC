import serial, time, sys
from reedsolo import RSCodec

PORT = "/dev/ttyAMA0"
BAUD = 38400

MODE_CFG = {
    "a": ("1/4", 7,  0),
    "b": ("1/3", 11, 1),
    "c": ("2/3", 11, 2),
}

MODE = "a"
PAYLOAD_RATE, PAYLOAD_RS_K, PAYLOAD_MCS_ID = MODE_CFG[MODE]

INTER_ROWS = 15

FLP = [1, 0] * 32
TDP = [0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0]
TDP_INV = [1 - b for b in TDP]
SHR_BITS = FLP + TDP + TDP_INV + TDP + TDP_INV

G = [0o133, 0o171, 0o165]
K = 7
MASK = (1 << K) - 1

def make_rs(k):
    return RSCodec(nsym=15-k, nsize=15, c_exp=4, prim=0x13, fcr=1, generator=2)

RS_7  = make_rs(7)
RS_11 = make_rs(11)

def get_rs(k):
    return RS_7 if k == 7 else RS_11

def pad_to_multiple(bits, m, pad=0):
    r = len(bits) % m
    return list(bits) if r == 0 else list(bits) + [pad] * (m - r)

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

def cc_encode_1_3(info_bits):
    state = 0
    out = []
    for b in list(info_bits) + [0] * (K - 1):
        state = ((state << 1) | (b & 1)) & MASK
        out.append(parity(state & G[0]))
        out.append(parity(state & G[1]))
        out.append(parity(state & G[2]))
    return out

def rate_map(mother_bits, rate):
    if rate == "1/3":
        return list(mother_bits)

    if rate == "2/3":
        out = []
        for i in range(0, len(mother_bits), 6):
            if i + 5 < len(mother_bits):
                out.extend([mother_bits[i], mother_bits[i+1], mother_bits[i+4]])
        return out

    if rate == "1/4":
        out = []
        for i in range(0, len(mother_bits), 6):
            if i + 5 < len(mother_bits):
                half = [mother_bits[i], mother_bits[i+1], mother_bits[i+3], mother_bits[i+4]]
                for b in half:
                    out.extend([b, b])
        return out

    raise ValueError("rate invalido")

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

def process_chain(bits, rate, rs_k):
    b_rs = rs16_encode(bits_to_bytes(bits), rs_k)
    b_rs = bytes_to_bits(b_rs)

    b_int = interleave_and_puncture(b_rs, INTER_ROWS)
    b_cc = cc_encode_1_3(b_int)

    if rate in ("1/4", "2/3"):
        b_cc = pad_to_multiple(b_cc, 6, 0)

    return rate_map(b_cc, rate)

def manchester(msg):
    seq = []
    for b in msg:
        if b == 1:
            seq.extend([0,1])
        else:
            seq.extend([1,0])
    return seq

def mseq():
    state = [1,0,0,0,0,0,0]
    seq = []
    L = 2**7 - 1
    for _ in range(L):
        seq.append(state[-1])
        new = state[0] ^ state[-1]
        state = [new] + state[:-1]
    return seq

def build_phr_bits(payload_len_bytes: int, payload_mcs_id: int):
    phr = [0, 0, 0, 0]
    phr.extend([(payload_mcs_id  >> i) & 1 for i in range(6)])
    phr.extend([(payload_len_bytes >> i) & 1 for i in range(16)])
    phr.extend([0] * 6)
    return phr

def build_packet(seq):
    HEADER_RS_K = 7
    HEADER_RATE = "1/4"

    seq_bytes = bits_to_bytes(seq)
    seq_bits  = bytes_to_bits(seq_bytes)

    phr_bits = build_phr_bits(len(seq_bytes), PAYLOAD_MCS_ID)
    hcs = crc16_hcs(phr_bits)
    header_bits = phr_bits + u16_to_bits(hcs)

    header_coded  = process_chain(header_bits, HEADER_RATE, HEADER_RS_K)
    payload_coded = process_chain(seq_bits,    PAYLOAD_RATE, PAYLOAD_RS_K)

    psdu_bits = SHR_BITS + header_coded + payload_coded
    tx_bits = manchester(psdu_bits)
    return bits_to_bytes(tx_bits)

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print("Error puerto:", e)
        sys.exit(1)

    print(f"TX Completo - MODO = {MODE} (rate={PAYLOAD_RATE}, rs_k={PAYLOAD_RS_K}, mcs_id={PAYLOAD_MCS_ID})")

    try:
        for i in range(200):
            msg = mseq()
            pkt = build_packet(msg)
            ser.write(pkt)
            ser.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Tx terminado")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
