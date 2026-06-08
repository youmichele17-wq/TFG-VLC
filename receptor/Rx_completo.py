
import serial, time, sys
from reedsolo import RSCodec, ReedSolomonError
from BER import calc_BER

PORT = "/dev/ttyAMA0"
BAUD = 38400
SYNC_MAX_ERR = 10
INTER_ROWS = 15

FLP = [1, 0] * 32
TDP = [0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0]
TDP_INV = [1 - b for b in TDP]
SHR_BITS = FLP + TDP + TDP_INV + TDP + TDP_INV

MCS_TO_CFG = {
    0: ("1/4", 7),
    1: ("1/3", 11),
    2: ("2/3", 11),
}

G = [0o133, 0o171, 0o165]
K = 7
MASK = (1 << K) - 1

IDLE_TIMEOUT = 0.5
MIN_BITS_TO_TRY = 400

def parity(x: int) -> int:
    p = 0
    while x:
        p ^= 1
        x &= x - 1
    return p

def make_rs(k):
    return RSCodec(nsym=15-k, nsize=15, c_exp=4, prim=0x13, fcr=1, generator=2)

RS_7  = make_rs(7)
RS_11 = make_rs(11)

def get_rs(k):
    return RS_7 if k == 7 else RS_11

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

def bits_to_u16(bits):
    v = 0
    for i, b in enumerate(bits[:16]):
        v |= (b & 1) << i
    return v

def unpack_bytes_to_nibbles(packed: bytes):
    out = []
    for b in packed:
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

def rs16_coded_len_bytes_shortened(orig_len_bytes: int, rs_k: int) -> int:
    L = orig_len_bytes * 2
    full = L // rs_k
    s = L % rs_k
    rs_n = 15
    rs_nsym = rs_n - rs_k
    coded_syms = full * rs_n + (0 if s == 0 else (s + rs_nsym))
    return (coded_syms + 1) // 2

def rs16_decode_shortened(rs_payload_bytes: bytes, orig_len_bytes: int, rs_k: int, strict=False) -> bytes:
    rs = get_rs(rs_k)
    rs_n = 15
    rs_nsym = rs_n - rs_k

    L = orig_len_bytes * 2
    full = L // rs_k
    s = L % rs_k

    coded_syms = unpack_bytes_to_nibbles(rs_payload_bytes)
    idx = 0
    out_data_syms = []

    for _ in range(full):
        cw = bytes(coded_syms[idx:idx+rs_n])
        idx += rs_n
        try:
            dec = rs.decode(cw)[0]
        except ReedSolomonError:
            if strict:
                return b""
            dec = cw[:rs_k]
        out_data_syms.extend(dec)

    if s != 0:
        recv_len = s + rs_nsym
        part = coded_syms[idx:idx+recv_len]
        pre = rs_k - s
        cw = bytes(([0] * pre) + part)
        try:
            dec = rs.decode(cw)[0]
        except ReedSolomonError:
            if strict:
                return b""
            dec = cw[:rs_k]
        out_data_syms.extend(dec[pre:])

    out_data_syms = out_data_syms[:L]
    return pack_nibbles_to_bytes(out_data_syms)[:orig_len_bytes]

def manchester(msg):
    seq = []
    for i in range(0, len(msg) - 1, 2):
        a = msg[i]
        b = msg[i + 1]
        if a == 0 and b == 1:
            seq.append(1)
        elif a == 1 and b == 0:
            seq.append(0)
        else:
            seq.append(2)
    return seq

def find_sync(pattern, buf):
    L = len(pattern)
    for i in range(len(buf) - L + 1):
        err = 0
        for j in range(L):
            if buf[i+j] != pattern[j]:
                err += 1
                if err > SYNC_MAX_ERR:
                    break
        else:
            return i
    return -1

def depuncture(bits, rate):
    if rate == "1/3":
        return list(bits)

    elif rate == "2/3":
        out = []
        i = 0
        while i + 2 < len(bits):
            out += [bits[i], bits[i+1], 2, 2, bits[i+2], 2]
            i += 3
        return out

    elif rate == "1/4":
        out = []
        half = []
        for i in range(0, len(bits) - 1, 2):
            b0, b1 = bits[i], bits[i+1]
            half.append(b0 if b0 == b1 else 2)
        j = 0
        while j + 3 < len(half):
            out += [half[j], half[j+1], 2, half[j+2], half[j+3], 2]
            j += 4
        return out

    raise ValueError("Rate invalido")

TRELLIS = [[None, None] for _ in range(64)]
for s in range(64):
    for u in (0, 1):
        reg = ((s << 1) | u) & MASK
        y = (parity(reg & G[0]), parity(reg & G[1]), parity(reg & G[2]))
        TRELLIS[s][u] = (reg & 0x3F, y)

def viterbi(bits):
    n = len(bits)//3
    bits = bits[:3*n]

    INF = 10**9
    path = [INF] * 64
    path[0] = 0

    prev_s = [[0] * 64 for _ in range(n)]
    prev_u = [[0] * 64 for _ in range(n)]

    for i in range(n):
        r = bits[3*i: 3*i + 3]
        newp = [INF] * 64

        for s in range(64):
            if path[s] >= INF:
                continue

            for u in (0,1):
                ns, y = TRELLIS[s][u]
                cost = 0
                for k in range(3):
                    if r[k] == 2:
                        continue
                    if r[k] != y[k]:
                        cost += 1
                m = path[s] + cost
                if m < newp[ns]:
                    newp[ns] = m
                    prev_s[i][ns] = s
                    prev_u[i][ns] = u
        path = newp

    st = min(range(64), key=lambda s: path[s])

    dec = []
    for i in range(n - 1, -1, -1):
        dec.append(prev_u[i][st])
        st = prev_s[i][st]
    dec.reverse()

    return dec[:-(K - 1)]

def coded_len(n_info_bits, rate):
    mother = 3 * (n_info_bits + (K - 1))

    if rate == "1/3":
        return mother
    if rate == "2/3":
        return (mother // 6) * 3
    if rate == "1/4":
        return (mother // 6) * 8
    raise ValueError("rate inválido")

def deinterleave_with_insertion(bits, original_len, rows=15):
    N = original_len
    if N == 0:
        return []
    cols = (N + rows - 1) // rows

    mask_padded = [1] * N + [0] * ((rows * cols) - N)
    m_mask = [mask_padded[r*cols:(r+1)*cols] for r in range(rows)]

    interleaved_mask = []
    for c in range(cols):
        for r in range(rows):
            interleaved_mask.append(m_mask[r][c])

    reconstructed = []
    bit_idx = 0
    for m in interleaved_mask:
        if m == 1:
            reconstructed.append(bits[bit_idx] if bit_idx < len(bits) else 0)
            bit_idx += 1
        else:
            reconstructed.append(0)

    m_data = [[0]*cols for _ in range(rows)]
    idx = 0
    for c in range(cols):
        for r in range(rows):
            m_data[r][c] = reconstructed[idx]
            idx += 1

    out = []
    for r in range(rows):
        out.extend(m_data[r])
    return out[:N]

def try_decode_one(raw_bits):
    if len(raw_bits) < MIN_BITS_TO_TRY:
        return (False, False, None, 0)

    raw_inv = [1 - b for b in raw_bits]

    d_n  = manchester(raw_bits)
    d_n1 = manchester(raw_bits[1:])
    d_i  = manchester(raw_inv)
    d_i1 = manchester(raw_inv[1:])

    opts = [
        (find_sync(SHR_BITS, d_n),  d_n,  0),
        (find_sync(SHR_BITS, d_n1), d_n1, 1),
        (find_sync(SHR_BITS, d_i),  d_i,  0),
        (find_sync(SHR_BITS, d_i1), d_i1, 1),
    ]

    found = [x for x in opts if x[0] != -1]
    if not found:
        return (False, False, None, 0)

    found.sort(key=lambda x: x[0])
    idx, frame, phase = found[0]
    ptr = idx + len(SHR_BITS)

    base_consumed = phase + (ptr * 2)

    HEADER_BYTES = 6
    HEADER_RS_K = 7
    HEADER_RATE = "1/4"

    header_rs_bytes_len = rs16_coded_len_bytes_shortened(HEADER_BYTES, HEADER_RS_K)
    header_rs_bits_len  = header_rs_bytes_len * 8

    hdr_len = coded_len(header_rs_bits_len, HEADER_RATE)
    if len(frame) < ptr + hdr_len:
        return (False, False, None, 0)

    hdr_coded = frame[ptr: ptr + hdr_len]
    hdr_bits_int = viterbi(depuncture(hdr_coded, HEADER_RATE))
    hdr_rs_bits = deinterleave_with_insertion(hdr_bits_int, header_rs_bits_len, rows=INTER_ROWS)
    hdr_rs_bytes = bits_to_bytes(hdr_rs_bits)[:header_rs_bytes_len]
    hdr_bytes = rs16_decode_shortened(hdr_rs_bytes, HEADER_BYTES, HEADER_RS_K, strict=False)

    if (not hdr_bytes) or len(hdr_bytes) < 6:
        return (True, False, None, base_consumed)

    phr_bytes = hdr_bytes[:4]
    hcs_bytes = hdr_bytes[4:6]

    phr = bytes_to_bits(phr_bytes)[:32]
    tx_hcs = bits_to_u16(bytes_to_bits(hcs_bytes)[:16])
    calc_hcs = crc16_hcs(phr)

    if calc_hcs != tx_hcs:
        return (True, False, None, base_consumed)

    mcs = sum((phr[4+i] & 1) << i for i in range(6))
    length = sum((phr[10+i] & 1) << i for i in range(16))

    if any(phr[0:4]) or any(phr[26:32]):
        return (True, False, None, base_consumed)

    if mcs not in MCS_TO_CFG:
        return (True, False, None, base_consumed)

    rate, rs_k = MCS_TO_CFG[mcs]

    ptr_pl = ptr + hdr_len
    psdu_rs_bytes_len = rs16_coded_len_bytes_shortened(length, rs_k)
    psdu_rs_bits_len  = psdu_rs_bytes_len * 8
    psdu_len = coded_len(psdu_rs_bits_len, rate)

    if len(frame) < ptr_pl + psdu_len:
        return (False, False, None, 0)

    psdu_coded = frame[ptr_pl: ptr_pl + psdu_len]
    psdu_bits_int = viterbi(depuncture(psdu_coded, rate))
    psdu_rs_bits = deinterleave_with_insertion(psdu_bits_int, psdu_rs_bits_len, rows=INTER_ROWS)
    psdu_rs_bytes = bits_to_bytes(psdu_rs_bits)[:psdu_rs_bytes_len]
    msg = rs16_decode_shortened(psdu_rs_bytes, length, rs_k, strict=False)

    try:
        ber = calc_BER(bytes_to_bits(msg))
    except Exception:
        ber = None

    consumed = phase + (ptr_pl + psdu_len) * 2
    return (True, True, ber, consumed)

def main():
    trama = 0
    tramas_correctas = 0
    ber_acumulado = 0.0
    tramas_con_ber = 0

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.01)
    except Exception as e:
        print("Error puerto:", e)
        sys.exit(1)

    raw_bits = []
    last_data_ts = time.time()
    print("RX - Completo")

    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                raw_bits.extend(bytes_to_bits(chunk))
                last_data_ts = time.time()

            if len(raw_bits) < MIN_BITS_TO_TRY:
                if (time.time() - last_data_ts) > IDLE_TIMEOUT and len(raw_bits) > 0:
                    raw_bits = []
                    last_data_ts = time.time()
                continue

            captured, header_ok, ber, consumed = try_decode_one(raw_bits)

            if not captured:
                if len(raw_bits) > 200000:
                    raw_bits = raw_bits[len(raw_bits)//2:]
                continue

            trama += 1

            if header_ok:
                tramas_correctas += 1
                if ber is not None:
                    ber_acumulado += ber
                    tramas_con_ber += 1

                ber_medio = (ber_acumulado / tramas_con_ber) if tramas_con_ber else float('nan')

                print(f"Trama: {trama}")
                print(f"Tramas correctas: {tramas_correctas}")
                print(f"BER: {ber}")
                print(f"BER total: {ber_medio}")
            else:
                print(f"Trama: {trama}")
                print(f"Tramas correctas: {tramas_correctas}")
                print("Trama descartada (header/HCS KO)")

            if consumed > 0:
                raw_bits = raw_bits[consumed:]
            else:
                raw_bits = raw_bits[8:]

    except KeyboardInterrupt:
        pass
    finally:
        print("\n=== RESUMEN ===")
        print(f"Tramas captadas   : {trama}")
        print(f"Tramas correctas  : {tramas_correctas}")
        if tramas_con_ber > 0:
            print(f"BER total (media sobre {tramas_con_ber} tramas): {ber_acumulado / tramas_con_ber}")
        else:
            print("BER total: N/A (ninguna trama con BER calculado)")
        ser.close()

if __name__ == "__main__":
    main()