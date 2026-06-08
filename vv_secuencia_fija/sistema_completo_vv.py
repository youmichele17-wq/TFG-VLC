import random
from typing import List, Tuple, Optional
from reedsolo import RSCodec, ReedSolomonError

PAYLOAD_RATE = "2/3"

RS_SYM_ERR_HEADER = 4
BIT_ERR_HEADER = 90

RS_SYM_ERR_PAYLOAD = 0
BIT_ERR_PAYLOAD = 75

SEED = 2025

PAYLOAD_TEXT = (
    "El sol brillaba intensamente sobre el valle, mientras el viento suave "
    "movía las hojas y las aves cantaban alegremente alrededor."
)
PAYLOAD_BYTES = PAYLOAD_TEXT.encode("utf-8")

FLP = [1, 0] * 32
TDP = [0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0]
TDP_INV = [1 - b for b in TDP]
SYNC_FULL = FLP + TDP + TDP_INV + TDP + TDP_INV

INTER_ROWS = 15

HEADER_RATE = "1/4"
HEADER_RS_K = 7
HEADER_BYTES = 6

RATE_TO_MCS = {"1/4": 0, "1/3": 1, "2/3": 2}
MCS_TO_CFG = {
    0: ("1/4", 7),
    1: ("1/3", 11),
    2: ("2/3", 11)
}

G = [0o133, 0o171, 0o165]
K = 7
MASK = (1 << K) - 1


def parity(x: int) -> int:
    p = 0
    while x:
        p ^= 1
        x &= x - 1
    return p


def cc_encode_with_tail(info_bits: List[int]) -> List[int]:
    bits_in = list(info_bits) + [0] * (K - 1)
    state = 0
    out = []

    for b in bits_in:
        state = ((state << 1) | (b & 1)) & MASK
        out.append(parity(state & G[0]))
        out.append(parity(state & G[1]))
        out.append(parity(state & G[2]))

    return out


def rate_map(mother_bits: List[int], rate: str) -> List[int]:
    if rate == "1/3":
        return list(mother_bits)

    if rate == "2/3":
        out = []

        for i in range(0, len(mother_bits), 6):
            if i + 5 < len(mother_bits):
                out.extend([
                    mother_bits[i],
                    mother_bits[i + 1],
                    mother_bits[i + 4]
                ])
            elif i + 2 < len(mother_bits):
                out.extend([
                    mother_bits[i],
                    mother_bits[i + 1]
                ])

        return out

    if rate == "1/4":
        out_half = []

        for i in range(0, len(mother_bits), 6):
            if i + 5 < len(mother_bits):
                out_half.extend([
                    mother_bits[i],
                    mother_bits[i + 1],
                    mother_bits[i + 3],
                    mother_bits[i + 4]
                ])
            elif i + 2 < len(mother_bits):
                out_half.extend([
                    mother_bits[i],
                    mother_bits[i + 1]
                ])

        out = []

        for b in out_half:
            out.extend([b, b])

        return out

    raise ValueError("RATE inválido")


TRELLIS = [[None, None] for _ in range(64)]

for s in range(64):
    for u in (0, 1):
        reg = ((s << 1) | u) & MASK
        y = (
            parity(reg & G[0]),
            parity(reg & G[1]),
            parity(reg & G[2])
        )
        TRELLIS[s][u] = (reg & 0x3F, y)


def depuncture_erasures(bits: List[int], rate: str) -> List[int]:
    if rate == "1/3":
        return list(bits)

    if rate == "2/3":
        out = []
        i = 0

        while i + 2 < len(bits):
            A0, B0, B1 = bits[i], bits[i + 1], bits[i + 2]
            out.extend([A0, B0, -1])
            out.extend([-1, B1, -1])
            i += 3

        if i + 1 < len(bits):
            out.extend([
                bits[i],
                bits[i + 1],
                -1
            ])

        return out

    if rate == "1/4":
        half = []

        for i in range(0, len(bits) - 1, 2):
            b0, b1 = bits[i], bits[i + 1]
            half.append(b0 if b0 == b1 else -1)

        out = []
        i = 0

        while i + 3 < len(half):
            A0, B0, A1, B1 = half[i], half[i + 1], half[i + 2], half[i + 3]
            out.extend([A0, B0, -1])
            out.extend([A1, B1, -1])
            i += 4

        if i + 1 < len(half):
            out.extend([
                half[i],
                half[i + 1],
                -1
            ])

        return out

    raise ValueError("rate inválido")


def viterbi_mother_erasures(rx_syms: List[int]) -> List[int]:
    n = len(rx_syms) // 3
    rx_syms = rx_syms[:3 * n]

    INF = 10**9
    path = [INF] * 64
    path[0] = 0

    prev_s = [[0] * 64 for _ in range(n)]
    prev_u = [[0] * 64 for _ in range(n)]

    for t in range(n):
        r = rx_syms[3 * t:3 * t + 3]
        newp = [INF] * 64

        for s in range(64):
            if path[s] >= INF:
                continue

            for u in (0, 1):
                ns, y = TRELLIS[s][u]
                cost = 0

                for i in range(3):
                    if r[i] != -1 and r[i] != y[i]:
                        cost += 1

                m = path[s] + cost

                if m < newp[ns]:
                    newp[ns] = m
                    prev_s[t][ns] = s
                    prev_u[t][ns] = u

        path = newp

    st = min(range(64), key=lambda i: path[i])
    dec = []

    for t in range(n - 1, -1, -1):
        dec.append(prev_u[t][st])
        st = prev_s[t][st]

    dec.reverse()

    if len(dec) >= (K - 1):
        dec = dec[:-(K - 1)]

    return dec


def interleave_pad_and_puncture(bits: List[int], rows: int = 15) -> Tuple[List[int], int]:
    N = len(bits)
    cols = (N + rows - 1) // rows if N else 0

    if N == 0:
        return [], 0

    L = rows * cols
    pad_len = L - N

    bpad = list(bits) + [0] * pad_len
    m = [bpad[r * cols:(r + 1) * cols] for r in range(rows)]

    out = []

    for c in range(cols):
        for r in range(rows):
            idx_orig = r * cols + c
            if idx_orig < N:
                out.append(m[r][c])

    return out, cols


def interleave_mask(N: int, rows: int = 15) -> List[int]:
    cols = (N + rows - 1) // rows
    L = rows * cols
    pad_len = L - N

    mpad = [1] * N + [0] * pad_len
    m = [mpad[r * cols:(r + 1) * cols] for r in range(rows)]

    out = []

    for c in range(cols):
        for r in range(rows):
            out.append(m[r][c])

    return out


def reinsert_padding_bits(interleaved_punctured_bits: List[int], N: int, rows: int = 15) -> List[int]:
    mask = interleave_mask(N, rows)
    out = [0] * len(mask)
    j = 0

    for i in range(len(mask)):
        if mask[i] == 1:
            out[i] = interleaved_punctured_bits[j] if j < len(interleaved_punctured_bits) else 0
            j += 1

    return out


def deinterleave_15xD(bits: List[int], rows: int = 15) -> List[int]:
    if not bits:
        return []

    cols = (len(bits) + rows - 1) // rows
    need = rows * cols

    b = list(bits) + [0] * (need - len(bits))
    m = [[0] * cols for _ in range(rows)]

    idx = 0

    for c in range(cols):
        for r in range(rows):
            m[r][c] = b[idx]
            idx += 1

    out = []

    for r in range(rows):
        out.extend(m[r])

    return out


def bytes_to_bits_lsb(data: bytes) -> List[int]:
    return [((b >> i) & 1) for b in data for i in range(8)]


def bits_to_bytes_lsb(bits: List[int]) -> bytes:
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


def crc16_hcs_lsb(bits: List[int], init: int = 0xFFFF) -> int:
    crc = init
    poly = 0x8408

    for bit in bits:
        fb = (crc ^ (bit & 1)) & 1
        crc >>= 1

        if fb:
            crc ^= poly

    return crc & 0xFFFF


def u16_to_bits_lsb(x: int) -> List[int]:
    return [(x >> i) & 1 for i in range(16)]


def bits16_lsb_to_u16(bits16: List[int]) -> int:
    v = 0

    for i, b in enumerate(bits16[:16]):
        v |= (b & 1) << i

    return v


def build_phr_bits(payload_len_bytes: int, payload_mcs: int) -> List[int]:
    phr = []
    phr.append(0)
    phr.extend([0, 0, 0])
    phr.extend([(payload_mcs >> i) & 1 for i in range(6)])
    phr.extend([(payload_len_bytes >> i) & 1 for i in range(16)])
    phr.append(0)
    phr.extend([0, 0, 0, 0, 0])
    return phr


def parse_phr_32_lsb(bits32: List[int]) -> Optional[Tuple[int, int]]:
    if len(bits32) < 32:
        return None

    mcs = 0

    for i, b in enumerate(bits32[4:10]):
        mcs |= (b & 1) << i

    length = 0

    for i, b in enumerate(bits32[10:26]):
        length |= (b & 1) << i

    return mcs, length


def make_rs(k: int) -> RSCodec:
    return RSCodec(
        nsym=15 - k,
        nsize=15,
        c_exp=4,
        prim=0x13,
        fcr=1,
        generator=2
    )


RS_7 = make_rs(7)
RS_11 = make_rs(11)


def get_rs(k: int) -> RSCodec:
    return RS_7 if k == 7 else RS_11


def bytes_to_nibbles(data: bytes) -> List[int]:
    out = []

    for b in data:
        out.append((b >> 4) & 0xF)
        out.append(b & 0xF)

    return out


def pack_nibbles_to_bytes(nibs: List[int]) -> bytes:
    out = bytearray()

    for i in range(0, len(nibs), 2):
        hi = nibs[i] & 0xF
        lo = (nibs[i + 1] & 0xF) if i + 1 < len(nibs) else 0
        out.append((hi << 4) | lo)

    return bytes(out)


def unpack_bytes_to_nibbles(packed: bytes) -> List[int]:
    out = []

    for b in packed:
        out.append((b >> 4) & 0xF)
        out.append(b & 0xF)

    return out


def rs16_encode_shortened(payload_bytes: bytes, rs_k: int) -> bytes:
    rs = get_rs(rs_k)
    data_syms = bytes_to_nibbles(payload_bytes)

    coded_syms = []
    i = 0

    while i < len(data_syms):
        blk = data_syms[i:i + rs_k]
        s = len(blk)

        if s == rs_k:
            coded_syms.extend(rs.encode(bytes(blk)))
        else:
            pre = rs_k - s
            msg = bytes(([0] * pre) + blk)
            enc = rs.encode(msg)
            coded_syms.extend(enc[pre:])

        i += rs_k

    return pack_nibbles_to_bytes(coded_syms)


def rs16_decode_shortened(rs_payload_bytes: bytes, orig_len_bytes: int, rs_k: int, strict: bool = True) -> bytes:
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
        cw = bytes(coded_syms[idx:idx + rs_n])
        idx += rs_n
        dec = rs.decode(cw)[0]
        out_data_syms.extend(dec)

    if s != 0:
        recv_len = s + rs_nsym
        part = coded_syms[idx:idx + recv_len]
        pre = rs_k - s
        cw = bytes(([0] * pre) + part)
        dec = rs.decode(cw)[0]
        out_data_syms.extend(dec[pre:])

    out_data_syms = out_data_syms[:L]

    return pack_nibbles_to_bytes(out_data_syms)[:orig_len_bytes]


def flip_random_bits(bits: List[int], n: int, rng: random.Random) -> List[int]:
    if n <= 0:
        return list(bits)

    n = min(n, len(bits))
    pos = rng.sample(range(len(bits)), n)

    out = list(bits)

    for p in pos:
        out[p] ^= 1

    return out


def corrupt_rs_input_symbols_per_cw(rs_bits_in: List[int], v_per_cw: int, rng: random.Random) -> List[int]:
    if v_per_cw <= 0:
        return list(rs_bits_in)

    rs_bytes = bits_to_bytes_lsb(rs_bits_in)
    nibs = unpack_bytes_to_nibbles(rs_bytes)

    out = list(nibs)

    for base in range(0, len(out), 15):
        cw = out[base:base + 15]

        if len(cw) < 15:
            break

        pos = rng.sample(range(15), min(v_per_cw, 15))

        for p in pos:
            old = cw[p] & 0xF
            new = old

            while new == old:
                new = rng.randrange(16)

            cw[p] = new

        out[base:base + 15] = cw

    rs_bytes_cor = pack_nibbles_to_bytes(out)
    rs_bits_cor = bytes_to_bits_lsb(rs_bytes_cor)

    return rs_bits_cor[:len(rs_bits_in)]


def tx_encode_block(data_bytes: bytes, rs_k: int, rate: str) -> Tuple[bytes, List[int], int]:
    rs_bytes = rs16_encode_shortened(data_bytes, rs_k)
    rs_bits = bytes_to_bits_lsb(rs_bytes)

    cc_in, _ = interleave_pad_and_puncture(rs_bits, rows=INTER_ROWS)
    mother = cc_encode_with_tail(cc_in)
    coded = rate_map(mother, rate)

    return rs_bytes, coded, len(rs_bits)


def rx_decode_to_rs_bits(coded_bits: List[int], rs_bits_len: int, rate: str) -> List[int]:
    mother = depuncture_erasures(coded_bits, rate)
    dec_inter_punct = viterbi_mother_erasures(mother)

    inter_full = reinsert_padding_bits(
        dec_inter_punct,
        rs_bits_len,
        rows=INTER_ROWS
    )

    deint_full = deinterleave_15xD(
        inter_full,
        rows=INTER_ROWS
    )

    return deint_full[:rs_bits_len]


def main():
    rng = random.Random(SEED)

    if PAYLOAD_RATE not in RATE_TO_MCS:
        raise ValueError("PAYLOAD_RATE inválido")

    payload_mcs = RATE_TO_MCS[PAYLOAD_RATE]
    payload_rate, payload_rs_k = MCS_TO_CFG[payload_mcs]

    phr_bits = build_phr_bits(len(PAYLOAD_BYTES), payload_mcs)
    hcs = crc16_hcs_lsb(phr_bits, init=0xFFFF)

    header_bits = phr_bits + u16_to_bits_lsb(hcs)
    header_bytes = bits_to_bytes_lsb(header_bits)[:HEADER_BYTES]

    hdr_rs_ref, hdr_coded, hdr_rs_bits_len = tx_encode_block(
        header_bytes,
        HEADER_RS_K,
        HEADER_RATE
    )

    psdu_rs_ref, psdu_coded, psdu_rs_bits_len = tx_encode_block(
        PAYLOAD_BYTES,
        payload_rs_k,
        payload_rate
    )

    hdr_coded_ch = flip_random_bits(
        hdr_coded,
        BIT_ERR_HEADER,
        rng
    )

    psdu_coded_ch = flip_random_bits(
        psdu_coded,
        BIT_ERR_PAYLOAD,
        rng
    )

    hdr_rs_bits_in = rx_decode_to_rs_bits(
        hdr_coded_ch,
        hdr_rs_bits_len,
        HEADER_RATE
    )

    psdu_rs_bits_in = rx_decode_to_rs_bits(
        psdu_coded_ch,
        psdu_rs_bits_len,
        payload_rate
    )

    if RS_SYM_ERR_HEADER > 0:
        hdr_rs_bits_in = corrupt_rs_input_symbols_per_cw(
            hdr_rs_bits_in,
            RS_SYM_ERR_HEADER,
            rng
        )

    if RS_SYM_ERR_PAYLOAD > 0:
        psdu_rs_bits_in = corrupt_rs_input_symbols_per_cw(
            psdu_rs_bits_in,
            RS_SYM_ERR_PAYLOAD,
            rng
        )

    try:
        hdr_rs_bytes_in = bits_to_bytes_lsb(hdr_rs_bits_in)[:len(hdr_rs_ref)]
        hdr_bytes = rs16_decode_shortened(
            hdr_rs_bytes_in,
            HEADER_BYTES,
            HEADER_RS_K,
            strict=True
        )
        ok_hdr_rs = True
    except ReedSolomonError:
        ok_hdr_rs = False
        hdr_bytes = b""

    phr_ok = False

    if ok_hdr_rs and len(hdr_bytes) >= 6:
        phr_bytes = hdr_bytes[:4]
        hcs_bytes = hdr_bytes[4:6]

        phr_bits32 = bytes_to_bits_lsb(phr_bytes)[:32]
        got_hcs = bits16_lsb_to_u16(bytes_to_bits_lsb(hcs_bytes)[:16])
        calc_hcs = crc16_hcs_lsb(phr_bits32, init=0xFFFF)

        phr_ok = (got_hcs == calc_hcs)

    try:
        psdu_rs_bytes_in = bits_to_bytes_lsb(psdu_rs_bits_in)[:len(psdu_rs_ref)]
        payload_rx = rs16_decode_shortened(
            psdu_rs_bytes_in,
            len(PAYLOAD_BYTES),
            payload_rs_k,
            strict=True
        )
        ok_psdu_rs = True
    except ReedSolomonError:
        ok_psdu_rs = False
        payload_rx = b""

    ok_payload = ok_psdu_rs and (payload_rx == PAYLOAD_BYTES)
    resultado = ok_hdr_rs and phr_ok and ok_payload

    print(f"PAYLOAD_RATE={PAYLOAD_RATE}")
    print(f"BIT_ERR_HEADER={BIT_ERR_HEADER} | BIT_ERR_PAYLOAD={BIT_ERR_PAYLOAD}")
    print(f"RS_SYM_ERR_HEADER={RS_SYM_ERR_HEADER} | RS_SYM_ERR_PAYLOAD={RS_SYM_ERR_PAYLOAD}")
    print(f"RESULTADO: {'OK' if resultado else 'FAIL'}")


if __name__ == "__main__":
    main()