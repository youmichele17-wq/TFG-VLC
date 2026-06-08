import random
from typing import List, Tuple, Optional

SEED = 2025
N_TRIALS = 1
CRITERIO_100_OK = True

PAYLOAD_RATE = "1/4"
BIT_ERR_HEADER = 0
BIT_ERR_PAYLOAD = 0

PAYLOAD_TEXT = (
    "El sol brillaba intensamente sobre el valle, mientras el viento suave "
    "movía las hojas y las aves cantaban alegremente alrededor."
)

PAYLOAD_BYTES = PAYLOAD_TEXT.encode("utf-8")

INTER_ROWS = 15

FLP = [1, 0] * 32
TDP = [0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0]
TDP_INV = [1 - b for b in TDP]
SYNC_FULL = FLP + TDP + TDP_INV + TDP + TDP_INV

MCS_MAP = {"1/4": 0, "1/3": 1, "2/3": 2}
MCS_TO_RATE = {0: "1/4", 1: "1/3", 2: "2/3"}

G = [0o133, 0o171, 0o165]
K = 7
MASK = (1 << K) - 1


def parity(x: int) -> int:
    p = 0
    while x:
        p ^= 1
        x &= x - 1
    return p


def bytes_to_bits_lsb(data: bytes) -> List[int]:
    return [((b >> i) & 1) for b in data for i in range(8)]


def bits_to_bytes_lsb(bits: List[int]) -> bytes:
    b = list(bits)
    while len(b) % 8 != 0:
        b.append(0)

    out = bytearray()

    for i in range(0, len(b), 8):
        v = 0
        for j in range(8):
            v |= (b[i + j] & 1) << j
        out.append(v)

    return bytes(out)


def int_to_bits_lsb(val: int, n_bits: int) -> List[int]:
    return [(val >> i) & 1 for i in range(n_bits)]


def crc16_hcs_lsb(bits: List[int], init=0xFFFF) -> int:
    crc = init
    poly = 0x8408

    for bit in bits:
        fb = (crc ^ (bit & 1)) & 1
        crc >>= 1
        if fb:
            crc ^= poly

    return crc & 0xFFFF


def bits16_to_u16(bits: List[int]) -> int:
    v = 0

    for i, b in enumerate(bits[:16]):
        v |= (b & 1) << i

    return v


def interleave_and_puncture(bits: List[int], rows=15) -> List[int]:
    N = len(bits)

    if N == 0:
        return []

    cols = (N + rows - 1) // rows
    pad_len = (rows * cols) - N

    b_padded = list(bits) + [0] * pad_len
    mask_padded = [1] * N + [0] * pad_len

    matrix_data = [b_padded[r * cols:(r + 1) * cols] for r in range(rows)]
    matrix_mask = [mask_padded[r * cols:(r + 1) * cols] for r in range(rows)]

    out_bits = []

    for c in range(cols):
        for r in range(rows):
            if matrix_mask[r][c] == 1:
                out_bits.append(matrix_data[r][c])

    return out_bits


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


def rate_map(mother: List[int], rate: str) -> List[int]:
    if rate == "1/3":
        return list(mother)

    if rate == "2/3":
        out = []

        for i in range(0, len(mother), 6):
            if i + 5 < len(mother):
                out.extend([mother[i], mother[i + 1], mother[i + 4]])
            elif i + 2 < len(mother):
                out.extend([mother[i], mother[i + 1]])

        return out

    if rate == "1/4":
        half = []

        for i in range(0, len(mother), 6):
            if i + 5 < len(mother):
                half.extend([mother[i], mother[i + 1], mother[i + 3], mother[i + 4]])
            elif i + 2 < len(mother):
                half.extend([mother[i], mother[i + 1]])

        out = []

        for b in half:
            out.extend([b, b])

        return out

    raise ValueError("rate inválido")


def process_chain_tx(info_bits: List[int], rate: str) -> List[int]:
    b_int = interleave_and_puncture(info_bits, INTER_ROWS)
    b_cc = cc_encode_1_3(b_int)
    return rate_map(b_cc, rate)


def deinterleave_with_insertion(bits: List[int], original_len: int, rows=15) -> List[int]:
    N = original_len

    if N == 0:
        return []

    cols = (N + rows - 1) // rows

    mask_padded = [1] * N + [0] * ((rows * cols) - N)
    m_mask = [mask_padded[r * cols:(r + 1) * cols] for r in range(rows)]

    interleaved_mask = []

    for c in range(cols):
        for r in range(rows):
            interleaved_mask.append(m_mask[r][c])

    reconstructed = []
    bit_idx = 0

    for m in interleaved_mask:
        if m == 1:
            if bit_idx < len(bits):
                reconstructed.append(bits[bit_idx])
                bit_idx += 1
            else:
                reconstructed.append(0)
        else:
            reconstructed.append(0)

    m_data = [[0] * cols for _ in range(rows)]
    idx = 0

    for c in range(cols):
        for r in range(rows):
            m_data[r][c] = reconstructed[idx]
            idx += 1

    out = []

    for r in range(rows):
        out.extend(m_data[r])

    return out[:N]


TRELLIS = [[None, None] for _ in range(64)]

for s in range(64):
    for u in (0, 1):
        reg = ((s << 1) | u) & MASK
        y = (parity(reg & G[0]), parity(reg & G[1]), parity(reg & G[2]))
        TRELLIS[s][u] = (reg & 0x3F, y)


def depuncture_erasures(bits: List[int], rate: str) -> List[int]:
    if rate == "1/3":
        return list(bits)

    if rate == "2/3":
        out = []
        i = 0

        while i + 2 < len(bits):
            out.extend([bits[i], bits[i + 1], -1, -1, bits[i + 2], -1])
            i += 3

        if i + 1 < len(bits):
            out.extend([bits[i], bits[i + 1], -1])

        return out

    if rate == "1/4":
        out = []
        half = []

        for i in range(0, len(bits) - 1, 2):
            b0, b1 = bits[i], bits[i + 1]
            half.append(b0 if b0 == b1 else -1)

        i = 0

        while i + 3 < len(half):
            out.extend([half[i], half[i + 1], -1, half[i + 2], half[i + 3], -1])
            i += 4

        if i + 1 < len(half):
            out.extend([half[i], half[i + 1], -1])

        return out

    raise ValueError("rate inválido")


def viterbi(rx_syms: List[int]) -> List[int]:
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


def coded_len_punctured(info_bits: int, rate: str) -> int:
    cc_in = info_bits + (K - 1)

    if rate == "1/3":
        return 3 * cc_in

    if rate == "2/3":
        return 3 * (cc_in // 2) + (2 if (cc_in % 2) else 0)

    if rate == "1/4":
        return 8 * (cc_in // 2) + (4 if (cc_in % 2) else 0)

    raise ValueError("rate inválido")


def decode_chain_rx(coded_bits: List[int], rate: str, info_len_bits: int) -> List[int]:
    dec_bits = viterbi(depuncture_erasures(coded_bits, rate))
    return deinterleave_with_insertion(dec_bits, info_len_bits, INTER_ROWS)


def build_phr_bits(payload_len_bytes: int, mcs_id: int) -> List[int]:
    phr = [0] * 4
    phr.extend([(mcs_id >> i) & 1 for i in range(6)])
    phr.extend([(payload_len_bytes >> i) & 1 for i in range(16)])
    phr.extend([0] * 6)
    return phr


def parse_phr(bits32: List[int]) -> Optional[Tuple[int, int]]:
    if len(bits32) < 32:
        return None

    mcs = 0

    for i in range(6):
        mcs |= (bits32[4 + i] & 1) << i

    length = 0

    for i in range(16):
        length |= (bits32[10 + i] & 1) << i

    return mcs, length


def inject_errors_dual(
    frame_bits: List[int],
    hdr_off: int,
    hdr_len: int,
    psdu_off: int,
    psdu_len: int,
    n_hdr: int,
    n_psdu: int,
    rng: random.Random
) -> List[int]:
    rx = list(frame_bits)

    if n_hdr > 0 and hdr_len > 0:
        n = min(n_hdr, hdr_len)
        pos = rng.sample(range(hdr_off, hdr_off + hdr_len), n)

        for p in pos:
            rx[p] ^= 1

    if n_psdu > 0 and psdu_len > 0:
        n = min(n_psdu, psdu_len)
        pos = rng.sample(range(psdu_off, psdu_off + psdu_len), n)

        for p in pos:
            rx[p] ^= 1

    return rx


def run_trial(rng: random.Random) -> bool:
    if PAYLOAD_RATE not in MCS_MAP:
        raise ValueError("PAYLOAD_RATE inválido")

    mcs = MCS_MAP[PAYLOAD_RATE]

    phr = build_phr_bits(len(PAYLOAD_BYTES), mcs)
    hcs = crc16_hcs_lsb(phr)
    hdr_bits = phr + int_to_bits_lsb(hcs, 16)

    hdr_coded = process_chain_tx(hdr_bits, "1/4")

    psdu_bits = bytes_to_bits_lsb(PAYLOAD_BYTES)
    psdu_coded = process_chain_tx(psdu_bits, PAYLOAD_RATE)

    frame_tx = SYNC_FULL + hdr_coded + psdu_coded

    hdr_off = len(SYNC_FULL)
    hdr_len = len(hdr_coded)
    psdu_off = hdr_off + hdr_len
    psdu_len = len(psdu_coded)

    frame_rx = inject_errors_dual(
        frame_tx,
        hdr_off,
        hdr_len,
        psdu_off,
        psdu_len,
        BIT_ERR_HEADER,
        BIT_ERR_PAYLOAD,
        rng
    )

    ptr = len(SYNC_FULL)

    hdr_enc_len = coded_len_punctured(48, "1/4")

    if len(frame_rx) < ptr + hdr_enc_len:
        return False

    hdr_out = decode_chain_rx(frame_rx[ptr:ptr + hdr_enc_len], "1/4", 48)

    if len(hdr_out) < 48:
        return False

    phr_rx = hdr_out[:32]
    hcs_rx = bits16_to_u16(hdr_out[32:48])
    hcs_calc = crc16_hcs_lsb(phr_rx)

    parsed = parse_phr(phr_rx)

    if parsed is None or hcs_rx != hcs_calc:
        return False

    mcs_rx, length = parsed
    rate = MCS_TO_RATE.get(mcs_rx, "1/4")

    if not (0 < length <= 5000):
        return False

    p_bits = length * 8
    p_enc_len = coded_len_punctured(p_bits, rate)
    ptr_pl = ptr + hdr_enc_len

    if len(frame_rx) < ptr_pl + p_enc_len:
        return False

    psdu_out = decode_chain_rx(frame_rx[ptr_pl:ptr_pl + p_enc_len], rate, p_bits)
    msg = bits_to_bytes_lsb(psdu_out)[:length]

    return msg == PAYLOAD_BYTES


def main():
    rng = random.Random(SEED)
    ok_count = 0

    for _ in range(N_TRIALS):
        ok = run_trial(rng)

        if ok:
            ok_count += 1
        else:
            if CRITERIO_100_OK:
                break

    verdict = (ok_count == N_TRIALS) if CRITERIO_100_OK else (ok_count > 0)

    print(f"PAYLOAD_RATE={PAYLOAD_RATE}")
    print(f"BIT_ERR_HEADER={BIT_ERR_HEADER} | BIT_ERR_PAYLOAD={BIT_ERR_PAYLOAD}")
    print(f"RESULTADO: {'OK' if verdict else 'FAIL'}")


if __name__ == "__main__":
    main()