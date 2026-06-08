import random
from reedsolo import RSCodec, ReedSolomonError
from BER import calc_BER

INTER_ROWS = 15
SYNC_MAX_ERR = 10

FLP = [1, 0] * 32
TDP = [0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0]
TDP_INV = [1 - b for b in TDP]
SHR_BITS = FLP + TDP + TDP_INV + TDP + TDP_INV

def make_rs(k):
    return RSCodec(nsym=15 - k, nsize=15, c_exp=4, prim=0x13, fcr=1, generator=2)

RS_7 = make_rs(7)

RS_K_HEADER = 7
MCS_MODO_E = 4

G = [0o133, 0o171, 0o165]
K = 7
MASK = (1 << K) - 1

P_FLIP_HEADER = 0.13
P_FLIP = 0
P_END = 0.2
N = 200
base_seed = 1000

def ewgm(bits, p_flip, p_end, seed):
    error = random.Random(seed)
    corrupted = bits.copy()
    i = 0
    Ltot = len(corrupted)

    while i < Ltot:
        L = 1
        while error.random() > p_end:
            L += 1

        for j in range(i, min(i + L, Ltot)):
            if error.random() < p_flip:
                corrupted[j] ^= 1

        i += L

    return corrupted

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

def bits_to_u16(bits):
    v = 0

    for i, b in enumerate(bits[:16]):
        v |= (b & 1) << i

    return v

def u16_to_bits(x: int):
    return [(x >> i) & 1 for i in range(16)]

def crc16_hcs(bits, init=0xFFFF):
    crc = init
    poly = 0x8408

    for bit in bits:
        fb = (crc ^ (bit & 1)) & 1
        crc >>= 1

        if fb:
            crc ^= poly

    return crc & 0xFFFF

def mseq():
    state = [1, 0, 0, 0, 0, 0, 0]
    seq = []
    L = 2**7 - 1

    for _ in range(L):
        seq.append(state[-1])
        new = state[0] ^ state[-1]
        state = [new] + state[:-1]

    return seq

def manchester(bits):
    out = []

    for b in bits:
        if b == 0:
            out.extend([0, 1])
        else:
            out.extend([1, 0])

    return out

def manchester_decode(msg):
    out = []

    for i in range(0, len(msg) - 1, 2):
        a = msg[i]
        b = msg[i + 1]

        if a == 0 and b == 1:
            out.append(0)
        elif a == 1 and b == 0:
            out.append(1)
        else:
            out.append(2)

    return out

def find_sync(pattern, buf):
    L = len(pattern)

    for i in range(len(buf) - L + 1):
        err = 0

        for j in range(L):
            if buf[i + j] == 2:
                continue

            if buf[i + j] != pattern[j]:
                err += 1

                if err > SYNC_MAX_ERR:
                    break
        else:
            return i

    return -1

def build_phr_bits(payload_len_bytes: int, payload_mcs: int):
    phr = [0, 0, 0, 0]
    phr.extend([(payload_mcs >> i) & 1 for i in range(6)])
    phr.extend([(payload_len_bytes >> i) & 1 for i in range(16)])
    phr.extend([0] * 6)

    return phr

def bytes_to_nibbles(data: bytes):
    out = []

    for b in data:
        out.append((b >> 4) & 0xF)
        out.append(b & 0xF)

    return out

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
        lo = (nibs[i + 1] & 0xF) if (i + 1) < len(nibs) else 0
        out.append((hi << 4) | lo)
        i += 2

    return bytes(out)

def rs16_encode(payload_bytes: bytes, rs_k: int) -> bytes:
    rs = RS_7
    data_syms = bytes_to_nibbles(payload_bytes)
    coded_syms = []
    i = 0

    while i < len(data_syms):
        blk = data_syms[i:i + rs_k]
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

def rs16_coded_len_bytes_shortened(orig_len_bytes: int, rs_k: int) -> int:
    L = orig_len_bytes * 2
    full = L // rs_k
    s = L % rs_k
    rs_n = 15
    rs_nsym = rs_n - rs_k
    coded_syms = full * rs_n + (0 if s == 0 else (s + rs_nsym))

    return (coded_syms + 1) // 2

def rs16_decode_shortened(rs_payload_bytes: bytes, orig_len_bytes: int, rs_k: int, strict=True) -> bytes:
    rs = RS_7
    rs_n = 15
    rs_nsym = rs_n - rs_k

    L = orig_len_bytes * 2
    full = L // rs_k
    s = L % rs_k

    coded_syms = unpack_bytes_to_nibbles(rs_payload_bytes)
    idx = 0
    out_data_syms = []

    for _ in range(full):
        cw = bytes(coded_syms[idx:idx + rs_n])
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
        part = coded_syms[idx:idx + recv_len]
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

def interleave_and_puncture(bits, rows=15):
    Nbits = len(bits)

    if Nbits == 0:
        return []

    cols = (Nbits + rows - 1) // rows
    pad_len = (rows * cols) - Nbits

    b_padded = list(bits) + [0] * pad_len
    mask_padded = [1] * Nbits + [0] * pad_len

    matrix_data = [b_padded[r * cols:(r + 1) * cols] for r in range(rows)]
    matrix_mask = [mask_padded[r * cols:(r + 1) * cols] for r in range(rows)]

    out_bits = []

    for c in range(cols):
        for r in range(rows):
            if matrix_mask[r][c] == 1:
                out_bits.append(matrix_data[r][c])

    return out_bits

def deinterleave_with_insertion(bits, original_len, rows=15):
    Nbits = original_len

    if Nbits == 0:
        return []

    cols = (Nbits + rows - 1) // rows
    total = rows * cols

    mask_padded = [1] * Nbits + [0] * (total - Nbits)
    matrix_mask = [mask_padded[r * cols:(r + 1) * cols] for r in range(rows)]

    interleaved_mask = []

    for c in range(cols):
        for r in range(rows):
            interleaved_mask.append(matrix_mask[r][c])

    reconstructed = []
    bit_idx = 0

    for m in interleaved_mask:
        if m == 1:
            reconstructed.append(bits[bit_idx] if bit_idx < len(bits) else 0)
            bit_idx += 1
        else:
            reconstructed.append(0)

    matrix_data = [[0] * cols for _ in range(rows)]
    idx = 0

    for c in range(cols):
        for r in range(rows):
            matrix_data[r][c] = reconstructed[idx]
            idx += 1

    out = []

    for r in range(rows):
        out.extend(matrix_data[r])

    return out[:Nbits]

def cc_encode_1_3(info_bits):
    state = 0
    out = []

    for b in list(info_bits) + [0] * (K - 1):
        state = ((state << 1) | (b & 1)) & MASK
        out.append(parity(state & G[0]))
        out.append(parity(state & G[1]))
        out.append(parity(state & G[2]))

    return out

def pad_to_multiple(bits, m, pad=0):
    r = len(bits) % m

    if r == 0:
        return list(bits)

    return list(bits) + [pad] * (m - r)

def rate_map_1_4(mother_bits):
    out = []

    for i in range(0, len(mother_bits), 6):
        if i + 5 < len(mother_bits):
            selected = [
                mother_bits[i],
                mother_bits[i + 1],
                mother_bits[i + 3],
                mother_bits[i + 4],
            ]

            for b in selected:
                out.extend([b, b])

    return out

def encode_cc_1_4(info_bits):
    mother = cc_encode_1_3(info_bits)
    mother = pad_to_multiple(mother, 6, 0)

    return rate_map_1_4(mother)

def depuncture_1_4(bits):
    out = []
    half = []

    for i in range(0, len(bits) - 1, 2):
        b0, b1 = bits[i], bits[i + 1]
        half.append(b0 if b0 == b1 else 2)

    j = 0

    while j + 3 < len(half):
        A0, B0, A1, B1 = half[j], half[j + 1], half[j + 2], half[j + 3]
        out += [A0, B0, 2, A1, B1, 2]
        j += 4

    return out

TRELLIS = [[None, None] for _ in range(64)]

for s in range(64):
    for u in (0, 1):
        reg = ((s << 1) | u) & MASK
        y = (
            parity(reg & G[0]),
            parity(reg & G[1]),
            parity(reg & G[2]),
        )
        TRELLIS[s][u] = (reg & 0x3F, y)

def viterbi(bits):
    n = len(bits) // 3
    bits = bits[:3 * n]

    INF = 10**9
    path = [INF] * 64
    path[0] = 0

    prev_s = [[0] * 64 for _ in range(n)]
    prev_u = [[0] * 64 for _ in range(n)]

    for i in range(n):
        r = bits[3 * i:3 * i + 3]
        newp = [INF] * 64

        for s in range(64):
            if path[s] >= INF:
                continue

            for u in (0, 1):
                ns, y = TRELLIS[s][u]
                cost = 0

                for k in range(3):
                    if r[k] == 2:
                        continue

                    if r[k] != y[k]:
                        cost += 1

                metric = path[s] + cost

                if metric < newp[ns]:
                    newp[ns] = metric
                    prev_s[i][ns] = s
                    prev_u[i][ns] = u

        path = newp

    st = min(range(64), key=lambda s: path[s])

    dec = []

    for i in range(n - 1, -1, -1):
        dec.append(prev_u[i][st])
        st = prev_s[i][st]

    dec.reverse()

    if len(dec) >= (K - 1):
        return dec[:-(K - 1)]

    return []

def coded_len_1_4(n_info_bits):
    mother = 3 * (n_info_bits + (K - 1))
    mother = ((mother + 5) // 6) * 6

    return (mother // 6) * 8

def encode_header_chain(header_bits):
    b_rs = rs16_encode(bits_to_bytes(header_bits), RS_K_HEADER)
    b_rs_bits = bytes_to_bits(b_rs)
    b_int = interleave_and_puncture(b_rs_bits, INTER_ROWS)

    return encode_cc_1_4(b_int)

def build_packet(seq, seed):
    payload_mcs = MCS_MODO_E

    seq_bytes = bits_to_bytes(seq)
    payload_bits = bytes_to_bits(seq_bytes)

    phr_bits = build_phr_bits(len(seq_bytes), payload_mcs)
    hcs = crc16_hcs(phr_bits)
    header_bits = phr_bits + u16_to_bits(hcs)

    header_coded = encode_header_chain(header_bits)
    tail_bits = [0] * 6

    psdu_bits_tx = SHR_BITS + header_coded + tail_bits + payload_bits

    start_hdr = len(SHR_BITS)
    start_tail = start_hdr + len(header_coded)
    start_payload = start_tail + len(tail_bits)

    psdu_bits_rx = (
        psdu_bits_tx[:start_hdr]
        + ewgm(psdu_bits_tx[start_hdr:start_tail], P_FLIP_HEADER, P_END, seed + 12345)
        + psdu_bits_tx[start_tail:start_payload]
        + ewgm(psdu_bits_tx[start_payload:], P_FLIP, P_END, seed)
    )

    tx_bits = manchester(psdu_bits_rx)
    tx_bytes = bits_to_bytes(tx_bits)

    return (
        tx_bytes,
        payload_bits,
        header_coded,
        header_bits,
        psdu_bits_tx,
        psdu_bits_rx,
        start_tail,
        start_payload,
    )

def decode_header_chain(hdr_coded, header_rs_bits_len, header_rs_bytes_len):
    hdr_bits_int = viterbi(depuncture_1_4(hdr_coded))
    hdr_rs_bits = deinterleave_with_insertion(hdr_bits_int, header_rs_bits_len, rows=INTER_ROWS)
    hdr_rs_bytes = bits_to_bytes(hdr_rs_bits)[:header_rs_bytes_len]

    return rs16_decode_shortened(hdr_rs_bytes, 6, RS_K_HEADER, strict=True)

def main():
    print("V&V - Modo e")

    total = 0
    decoded = 0

    sum_post = 0.0
    sum_post_hdr = 0.0
    sum_pre_all = 0.0
    sum_pre_hdr_all = 0.0
    cnt_all = 0

    fail_hdr = 0
    fail_pl = 0
    ok_hdr = 0
    ok_pl = 0

    for t in range(N):
        seed = base_seed + t
        total += 1

        msg = mseq()

        (
            tx_bytes,
            payload_bits_tx,
            header_coded_tx,
            header_bits_tx,
            psdu_tx,
            psdu_rx,
            start_tail,
            start_payload,
        ) = build_packet(msg, seed)

        start_hdr = len(SHR_BITS)

        rx_hdr_pre = psdu_rx[start_hdr:start_tail]
        tx_hdr_pre = psdu_tx[start_hdr:start_tail]
        ber_pre_hdr_all = calc_BER(rx_hdr_pre, tx_hdr_pre, L=len(tx_hdr_pre))

        rx_payload_pre = psdu_rx[start_payload:start_payload + len(payload_bits_tx)]
        tx_payload_pre = psdu_tx[start_payload:start_payload + len(payload_bits_tx)]
        ber_pre_all = calc_BER(rx_payload_pre, tx_payload_pre, L=len(tx_payload_pre))

        sum_pre_all += ber_pre_all
        sum_pre_hdr_all += ber_pre_hdr_all
        cnt_all += 1

        raw_bits = bytes_to_bits(tx_bytes)

        if len(raw_bits) < 400:
            fail_hdr += 1
            continue

        d_n = manchester_decode(raw_bits)
        d_n1 = manchester_decode(raw_bits[1:])

        opts = [
            (find_sync(SHR_BITS, d_n), d_n, 0),
            (find_sync(SHR_BITS, d_n1), d_n1, 1),
        ]

        found = [x for x in opts if x[0] != -1]

        if not found:
            fail_hdr += 1
            continue

        found.sort(key=lambda x: x[0])
        idx, frame, phase = found[0]
        ptr = idx + len(SHR_BITS)

        HEADER_BYTES = 6

        header_rs_bytes_len = rs16_coded_len_bytes_shortened(HEADER_BYTES, RS_K_HEADER)
        header_rs_bits_len = header_rs_bytes_len * 8
        hdr_len = coded_len_1_4(header_rs_bits_len)

        if len(frame) < ptr + hdr_len:
            fail_hdr += 1
            continue

        hdr_coded = frame[ptr:ptr + hdr_len]
        hdr_bytes = decode_header_chain(hdr_coded, header_rs_bits_len, header_rs_bytes_len)

        if (not hdr_bytes) or len(hdr_bytes) < HEADER_BYTES:
            fail_hdr += 1
            continue

        phr_bytes = hdr_bytes[:4]
        hcs_bytes = hdr_bytes[4:6]

        phr = bytes_to_bits(phr_bytes)[:32]
        tx_hcs = bits_to_u16(bytes_to_bits(hcs_bytes)[:16])
        calc_hcs = crc16_hcs(phr)

        if calc_hcs != tx_hcs:
            fail_hdr += 1
            continue

        mcs = sum((phr[4 + i] & 1) << i for i in range(6))
        length = sum((phr[10 + i] & 1) << i for i in range(16))

        if mcs != MCS_MODO_E:
            fail_hdr += 1
            continue

        ok_hdr += 1

        ptr_payload = ptr + hdr_len + 6
        payload_bits_len = length * 8

        if len(frame) < ptr_payload + payload_bits_len:
            fail_pl += 1
            continue

        payload_bits_rx = frame[ptr_payload:ptr_payload + payload_bits_len]
        payload_bits_ref = payload_bits_tx[:payload_bits_len]

        ber_post = calc_BER(payload_bits_rx, payload_bits_ref, L=payload_bits_len)

        hdr_bits_post = bytes_to_bits(hdr_bytes)[:48]
        hdr_bits_ref = header_bits_tx[:48]
        err_hdr = sum(a != b for a, b in zip(hdr_bits_post, hdr_bits_ref))
        ber_post_hdr = err_hdr / 48

        decoded += 1
        sum_post += ber_post
        sum_post_hdr += ber_post_hdr

        if ber_post != 0:
            fail_pl += 1
        else:
            ok_pl += 1

    fer_pl = fail_pl / ok_hdr if ok_hdr else 1.0
    fer_hdr = fail_hdr / total if total else 1.0
    fer_total = (fail_hdr + fail_pl) / total if total else 1.0

    ber_pre_avg = sum_pre_all / cnt_all if cnt_all else 1.0
    ber_pre_hdr_avg = sum_pre_hdr_all / cnt_all if cnt_all else 1.0
    ber_post_avg = sum_post / decoded if decoded else 1.0
    ber_post_hdr_avg = sum_post_hdr / decoded if decoded else 1.0

    print(
        f"HEADER: P_FLIP:{P_FLIP_HEADER}  P_END:{P_END}  "
        f"BER_pre_avg:{ber_pre_hdr_avg:.6f}  BER_post_avg:{ber_post_hdr_avg:.6f}  "
        f"FER_hdr:{fer_hdr:.6f}"
    )

    print(
        f"PAYLOAD: P_FLIP:{P_FLIP}  "
        f"BER_pre_avg:{ber_pre_avg:.6f}  BER_post_avg:{ber_post_avg:.6f}  "
        f"FER_pl:{fer_pl:.6f}"
    )

    print(f"FER_total:{fer_total:.6f}")

if __name__ == "__main__":
    main()