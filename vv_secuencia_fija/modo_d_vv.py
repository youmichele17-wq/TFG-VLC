import random
from typing import List, Tuple, Optional
from reedsolo import RSCodec, ReedSolomonError


SEED = 2025

PAYLOAD_RATE = "d"

BIT_ERR_HEADER = 100

RS_SYM_ERR_HEADER = 0
RS_SYM_ERR_PAYLOAD = 2


PAYLOAD_TEXT = (
    "El sol brillaba intensamente sobre el valle, mientras el viento suave "
    "movía las hojas y las aves cantaban alegremente alrededor."
)

PAYLOAD_BYTES = PAYLOAD_TEXT.encode("utf-8")


MCS_ID_D = 3

HEADER_BYTES = 6
HEADER_RS_K = 7
PAYLOAD_RS_K = 11

RS_N = 15
INTER_ROWS = 15


G = [0o133, 0o171, 0o165]
K = 7
MASK = (1 << K) - 1


def make_rs(k: int):
    return RSCodec(
        nsym=RS_N - k,
        nsize=RS_N,
        c_exp=4,
        prim=0x13,
        fcr=1,
        generator=2
    )


RS_7 = make_rs(HEADER_RS_K)
RS_11 = make_rs(PAYLOAD_RS_K)


def get_rs(k: int):
    if k == HEADER_RS_K:
        return RS_7

    return RS_11


def parity(x: int) -> int:
    p = 0

    while x:
        p ^= 1
        x &= x - 1

    return p


def cc_encode_1_3(bits: List[int]) -> List[int]:
    bits_in = list(bits) + [0] * (K - 1)

    state = 0
    out = []

    for b in bits_in:
        state = ((state << 1) | (b & 1)) & MASK

        out.append(parity(state & G[0]))
        out.append(parity(state & G[1]))
        out.append(parity(state & G[2]))

    return out


def rate_map_1_4_from_mother(mother_bits: List[int]) -> List[int]:
    half = []

    for i in range(0, len(mother_bits), 6):
        if i + 5 < len(mother_bits):
            half.extend([
                mother_bits[i],
                mother_bits[i + 1],
                mother_bits[i + 3],
                mother_bits[i + 4]
            ])

        elif i + 2 < len(mother_bits):
            half.extend([
                mother_bits[i],
                mother_bits[i + 1]
            ])

    out = []

    for b in half:
        out.extend([b, b])

    return out


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


def depuncture_1_4_to_mother_erasures(bits: List[int]) -> List[int]:
    half = []

    for i in range(0, len(bits) - 1, 2):
        b0, b1 = bits[i], bits[i + 1]

        if b0 == b1:
            half.append(b0)
        else:
            half.append(-1)

    out = []
    i = 0

    while i + 3 < len(half):
        A0 = half[i]
        B0 = half[i + 1]
        A1 = half[i + 2]
        B1 = half[i + 3]

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


def u16_to_bits_lsb(x: int) -> List[int]:
    return [(x >> i) & 1 for i in range(16)]


def bits16_lsb_to_u16(bits16: List[int]) -> int:
    v = 0

    for i, b in enumerate(bits16[:16]):
        v |= (b & 1) << i

    return v


def crc16_hcs_lsb(bits: List[int], init: int = 0xFFFF) -> int:
    crc = init
    poly = 0x8408

    for bit in bits:
        fb = (crc ^ (bit & 1)) & 1

        crc >>= 1

        if fb:
            crc ^= poly

    return crc & 0xFFFF


def build_phr_bits(payload_len_bytes: int, mcs_id: int) -> List[int]:
    phr = [0, 0, 0, 0]

    phr.extend([(mcs_id >> i) & 1 for i in range(6)])
    phr.extend([(payload_len_bytes >> i) & 1 for i in range(16)])
    phr.extend([0, 0, 0, 0, 0, 0])

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


def bytes_to_nibbles_low_first(data: bytes) -> List[int]:
    out = []

    for b in data:
        out.append(b & 0xF)
        out.append((b >> 4) & 0xF)

    return out


def pack_nibbles_low_first_to_bytes(nibs: List[int]) -> bytes:
    out = bytearray()

    for i in range(0, len(nibs), 2):
        lo = nibs[i] & 0xF
        hi = (nibs[i + 1] & 0xF) if i + 1 < len(nibs) else 0

        out.append((hi << 4) | lo)

    return bytes(out)


def nibbles_to_bits_lsb(nibs: List[int]) -> List[int]:
    bits = []

    for s in nibs:
        s &= 0xF

        bits.extend([
            (s >> 0) & 1,
            (s >> 1) & 1,
            (s >> 2) & 1,
            (s >> 3) & 1
        ])

    return bits


def bits_to_nibbles_lsb(bits: List[int]) -> List[int]:
    bits = list(bits)

    while len(bits) % 4 != 0:
        bits.append(0)

    out = []

    for i in range(0, len(bits), 4):
        v = 0

        for j in range(4):
            v |= (bits[i + j] & 1) << j

        out.append(v & 0xF)

    return out


def rs_encode_shortened_syms(data_syms: List[int], rs_k: int) -> List[int]:
    rs = get_rs(rs_k)

    out = []
    i = 0

    while i < len(data_syms):
        blk = data_syms[i:i + rs_k]

        if len(blk) == rs_k:
            out.extend(list(rs.encode(bytes(blk))))
            i += rs_k

        else:
            rem = len(blk)
            pre = rs_k - rem

            msg = [0] * pre + list(blk)
            cw = list(rs.encode(bytes(msg)))

            out.extend(cw[pre:])

            break

    return out


def rs_decode_shortened_syms(
    coded_syms: List[int],
    orig_len_syms: int,
    rs_k: int,
    strict=True
) -> List[int]:
    rs = get_rs(rs_k)

    nsym = RS_N - rs_k

    full = orig_len_syms // rs_k
    rem = orig_len_syms % rs_k

    out = []
    idx = 0

    for _ in range(full):
        cw = coded_syms[idx:idx + RS_N]

        if len(cw) < RS_N:
            if strict:
                raise ReedSolomonError("short cw")

            return out[:orig_len_syms]

        dec = rs.decode(bytes(cw))[0]

        out.extend(list(dec))

        idx += RS_N

    if rem != 0:
        recv_len = rem + nsym

        part = coded_syms[idx:idx + recv_len]

        if len(part) < recv_len:
            if strict:
                raise ReedSolomonError("short partial cw")

            return out[:orig_len_syms]

        pre = rs_k - rem

        cw = [0] * pre + list(part)

        dec = rs.decode(bytes(cw))[0]

        out.extend(list(dec)[pre:])

    return out[:orig_len_syms]


def interleave_symbols_and_puncture(
    sym: List[int],
    rows: int = 15
) -> List[int]:
    S = len(sym)

    if S == 0:
        return []

    cols = (S + rows - 1) // rows

    total = rows * cols
    pad_len = total - S

    padded = list(sym) + [0] * pad_len
    mask = [1] * S + [0] * pad_len

    mat = [
        padded[r * cols:(r + 1) * cols]
        for r in range(rows)
    ]

    msk = [
        mask[r * cols:(r + 1) * cols]
        for r in range(rows)
    ]

    out = []

    for c in range(cols):
        for r in range(rows):
            if msk[r][c]:
                out.append(mat[r][c])

    return out


def deinterleave_symbols_unpuncture(
    sym_rx: List[int],
    S: int,
    rows: int = 15
) -> List[int]:
    if S == 0:
        return []

    cols = (S + rows - 1) // rows

    total = rows * cols
    pad_len = total - S

    mask = [1] * S + [0] * pad_len

    msk = [
        mask[r * cols:(r + 1) * cols]
        for r in range(rows)
    ]

    mat = [
        [0] * cols
        for _ in range(rows)
    ]

    idx = 0

    for c in range(cols):
        for r in range(rows):
            if msk[r][c]:
                if idx < len(sym_rx):
                    mat[r][c] = sym_rx[idx] & 0xF
                else:
                    mat[r][c] = 0

                idx += 1

    out = []

    for r in range(rows):
        out.extend(mat[r])

    return out[:S]


def flip_random_bits(
    bits: List[int],
    n: int,
    rng: random.Random
) -> List[int]:
    if n <= 0:
        return list(bits)

    out = list(bits)

    n = min(n, len(out))

    pos = rng.sample(range(len(out)), n)

    for p in pos:
        out[p] ^= 1

    return out


def corrupt_rs_symbols_per_cw(
    rs_syms_in: List[int],
    v_per_cw: int,
    rng: random.Random
) -> List[int]:
    if v_per_cw <= 0:
        return list(rs_syms_in)

    out = list(rs_syms_in)

    idx = 0

    while idx + RS_N <= len(out):
        cw = out[idx:idx + RS_N]

        v = min(v_per_cw, RS_N)

        pos = rng.sample(range(RS_N), v)

        for p in pos:
            old = cw[p] & 0xF
            new = old

            while new == old:
                new = rng.randrange(16)

            cw[p] = new

        out[idx:idx + RS_N] = cw

        idx += RS_N

    return out


def encode_header_mcs0(
    phr_bits_32: List[int]
) -> Tuple[List[int], int, int]:
    hcs = crc16_hcs_lsb(phr_bits_32)

    hdr_bits = list(phr_bits_32) + u16_to_bits_lsb(hcs)

    hdr_bytes = bits_to_bytes_lsb(hdr_bits)[:HEADER_BYTES]

    hdr_data_syms = bytes_to_nibbles_low_first(hdr_bytes)

    rs_syms = rs_encode_shortened_syms(
        hdr_data_syms,
        HEADER_RS_K
    )

    srs = len(rs_syms)

    ilv_syms = interleave_symbols_and_puncture(
        rs_syms,
        rows=INTER_ROWS
    )

    ilv_bits = nibbles_to_bits_lsb(ilv_syms)

    mother = cc_encode_1_3(ilv_bits)

    header_coded_bits = rate_map_1_4_from_mother(mother)

    return header_coded_bits, srs, len(hdr_data_syms)


def decode_header_to_rs_symbols(
    header_coded_bits: List[int],
    srs: int
) -> List[int]:
    mother = depuncture_1_4_to_mother_erasures(header_coded_bits)

    dec_bits = viterbi_mother_erasures(mother)

    rs_inter_syms = bits_to_nibbles_lsb(
        dec_bits[:srs * 4]
    )[:srs]

    rs_syms_in = deinterleave_symbols_unpuncture(
        rs_inter_syms,
        S=srs,
        rows=INTER_ROWS
    )

    return rs_syms_in


def encode_payload_d(
    data_bytes: bytes
) -> Tuple[List[int], int, int]:
    data_syms = bytes_to_nibbles_low_first(data_bytes)

    rs_syms = rs_encode_shortened_syms(
        data_syms,
        PAYLOAD_RS_K
    )

    srs = len(rs_syms)

    ilv_syms = interleave_symbols_and_puncture(
        rs_syms,
        rows=INTER_ROWS
    )

    return ilv_syms, srs, len(data_syms)


def decode_payload_d_to_rs_symbols(
    ilv_syms_rx: List[int],
    srs: int
) -> List[int]:
    return deinterleave_symbols_unpuncture(
        ilv_syms_rx,
        S=srs,
        rows=INTER_ROWS
    )


def main():
    rng = random.Random(SEED)

    phr_bits = build_phr_bits(
        len(PAYLOAD_BYTES),
        MCS_ID_D
    )

    header_coded_tx, hdr_srs, hdr_data_len_syms = encode_header_mcs0(
        phr_bits
    )

    payload_ilv_tx, psdu_srs, psdu_data_len_syms = encode_payload_d(
        PAYLOAD_BYTES
    )

    header_coded_rx = flip_random_bits(
        header_coded_tx,
        BIT_ERR_HEADER,
        rng
    )

    hdr_rs_syms_in = decode_header_to_rs_symbols(
        header_coded_rx,
        hdr_srs
    )

    psdu_rs_syms_in = decode_payload_d_to_rs_symbols(
        payload_ilv_tx,
        psdu_srs
    )

    hdr_rs_syms_in = corrupt_rs_symbols_per_cw(
        hdr_rs_syms_in,
        RS_SYM_ERR_HEADER,
        rng
    )

    psdu_rs_syms_in = corrupt_rs_symbols_per_cw(
        psdu_rs_syms_in,
        RS_SYM_ERR_PAYLOAD,
        rng
    )

    try:
        hdr_data_syms_rx = rs_decode_shortened_syms(
            hdr_rs_syms_in,
            orig_len_syms=hdr_data_len_syms,
            rs_k=HEADER_RS_K,
            strict=True
        )

        header_rx = pack_nibbles_low_first_to_bytes(
            hdr_data_syms_rx
        )[:HEADER_BYTES]

        ok_hdr_rs = True

    except ReedSolomonError:
        header_rx = b""
        ok_hdr_rs = False

    phr_ok = False

    if ok_hdr_rs and len(header_rx) >= HEADER_BYTES:
        phr_rx_bytes = header_rx[:4]
        hcs_rx_bytes = header_rx[4:6]

        phr_rx_bits = bytes_to_bits_lsb(
            phr_rx_bytes
        )[:32]

        got_hcs = bits16_lsb_to_u16(
            bytes_to_bits_lsb(hcs_rx_bytes)[:16]
        )

        calc_hcs = crc16_hcs_lsb(
            phr_rx_bits,
            init=0xFFFF
        )

        parsed = parse_phr_32_lsb(phr_rx_bits)

        if parsed is not None:
            mcs_rx, length_rx = parsed

            phr_ok = (
                got_hcs == calc_hcs
                and mcs_rx == MCS_ID_D
                and length_rx == len(PAYLOAD_BYTES)
            )

    try:
        psdu_data_syms_rx = rs_decode_shortened_syms(
            psdu_rs_syms_in,
            orig_len_syms=psdu_data_len_syms,
            rs_k=PAYLOAD_RS_K,
            strict=True
        )

        payload_rx = pack_nibbles_low_first_to_bytes(
            psdu_data_syms_rx
        )[:len(PAYLOAD_BYTES)]

        ok_payload = payload_rx == PAYLOAD_BYTES

    except ReedSolomonError:
        ok_payload = False

    resultado = ok_hdr_rs and phr_ok and ok_payload

    print(f"PAYLOAD_RATE={PAYLOAD_RATE}")
    print(f"BIT_ERR_HEADER={BIT_ERR_HEADER}")
    print(f"RS_SYM_ERR_HEADER={RS_SYM_ERR_HEADER} | RS_SYM_ERR_PAYLOAD={RS_SYM_ERR_PAYLOAD}")
    print()
    print(f"RESULTADO: {'OK' if resultado else 'FAIL'}")


if __name__ == "__main__":
    main()

