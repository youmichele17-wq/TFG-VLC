import random
from typing import List, Tuple, Optional

SEED = 2025
PAYLOAD_RATE = "1/4"
BIT_ERR_HEADER = 0
BIT_ERR_PAYLOAD = 600
N_TRIALS = 1
CRITERIO_100_OK = True

PAYLOAD_TEXT = (
    "El sol brillaba intensamente sobre el valle, mientras el viento suave "
    "movía las hojas y las aves cantaban alegremente alrededor."
)

PAYLOAD_BYTES = PAYLOAD_TEXT.encode("utf-8")

FLP = [1, 0] * 32
TDP = [0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0]
TDP_INV = [1 - b for b in TDP]
SYNC_FULL = FLP + TDP + TDP_INV + TDP + TDP_INV

RATE_TO_MCS = {"1/4": 0, "1/3": 1, "2/3": 2}
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

def crc16_hcs_lsb(bits: List[int], init=0xFFFF) -> int:
    crc = init
    poly = 0x8408
    for bit in bits:
        fb = (crc ^ (bit & 1)) & 1
        crc >>= 1
        if fb:
            crc ^= poly
    return crc & 0xFFFF

def cc_encode_1_3(info_bits: List[int]) -> List[int]:
    state = 0
    out = []
    for b in list(info_bits) + [0] * (K - 1):
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
                out.extend([mother_bits[i], mother_bits[i+1], mother_bits[i+4]])
            elif i + 2 < len(mother_bits):
                out.extend([mother_bits[i], mother_bits[i+1]])
        return out

    if rate == "1/4":
        half = []
        for i in range(0, len(mother_bits), 6):
            if i + 5 < len(mother_bits):
                half.extend([mother_bits[i], mother_bits[i+1], mother_bits[i+3], mother_bits[i+4]])
            elif i + 2 < len(mother_bits):
                half.extend([mother_bits[i], mother_bits[i+1]])

        out = []
        for b in half:
            out.extend([b, b])
        return out

    raise ValueError("rate inválido")

def encode_bits_cc(info_bits: List[int], rate: str) -> List[int]:
    return rate_map(cc_encode_1_3(info_bits), rate)

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
            A0, B0, B1 = bits[i], bits[i+1], bits[i+2]
            out.extend([A0, B0, -1])
            out.extend([-1, B1, -1])
            i += 3
        if i + 1 < len(bits):
            out.extend([bits[i], bits[i+1], -1])
        return out

    if rate == "1/4":
        half = []
        for i in range(0, len(bits) - 1, 2):
            b0, b1 = bits[i], bits[i+1]
            half.append(b0 if b0 == b1 else -1)

        out = []
        i = 0
        while i + 3 < len(half):
            A0, B0, A1, B1 = half[i], half[i+1], half[i+2], half[i+3]
            out.extend([A0, B0, -1])
            out.extend([A1, B1, -1])
            i += 4

        if i + 1 < len(half):
            out.extend([half[i], half[i+1], -1])

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
        r = rx_syms[3*t:3*t+3]
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

def coded_bits_len_for_info(info_bits: int, rate: str) -> int:
    m = info_bits + (K - 1)

    if rate == "1/3":
        return 3 * m

    if rate == "2/3":
        return 3 * (m // 2) + (2 if (m % 2) else 0)

    if rate == "1/4":
        return 8 * (m // 2) + (4 if (m % 2) else 0)

    raise ValueError("rate inválido")

def build_phr_bits(payload_len_bytes: int, payload_mcs: int) -> List[int]:
    phr = [0, 0, 0, 0]
    phr.extend([(payload_mcs >> i) & 1 for i in range(6)])
    phr.extend([(payload_len_bytes >> i) & 1 for i in range(16)])
    phr.extend([0] * 6)
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

def flip_in_segment(bits: List[int], start: int, length: int, n_flips: int, rng: random.Random) -> None:
    if n_flips <= 0 or length <= 0:
        return

    n = min(n_flips, length)
    pos = rng.sample(range(start, start + length), n)

    for p in pos:
        bits[p] ^= 1

def run_trial(n_hdr: int, n_psdu: int, rng: random.Random) -> bool:
    if PAYLOAD_RATE not in RATE_TO_MCS:
        raise ValueError("PAYLOAD_RATE inválido")

    payload_mcs = RATE_TO_MCS[PAYLOAD_RATE]

    phr_bits = build_phr_bits(len(PAYLOAD_BYTES), payload_mcs)
    hcs = crc16_hcs_lsb(phr_bits, init=0xFFFF)
    header_bits = phr_bits + u16_to_bits_lsb(hcs)

    header_coded = encode_bits_cc(header_bits, "1/4")
    payload_bits = bytes_to_bits_lsb(PAYLOAD_BYTES)
    payload_coded = encode_bits_cc(payload_bits, PAYLOAD_RATE)

    frame_tx = SYNC_FULL + header_coded + payload_coded

    hdr_off = len(SYNC_FULL)
    hdr_len = len(header_coded)
    psdu_off = hdr_off + hdr_len
    psdu_len = len(payload_coded)

    rx = list(frame_tx)

    flip_in_segment(rx, hdr_off, hdr_len, n_hdr, rng)
    flip_in_segment(rx, psdu_off, psdu_len, n_psdu, rng)

    ptr = len(SYNC_FULL)

    HDR_INFO_BITS = 48
    HDR_RATE = "1/4"

    hdr_coded_len = coded_bits_len_for_info(HDR_INFO_BITS, HDR_RATE)
    hdr_coded_rx = rx[ptr:ptr + hdr_coded_len]

    hdr_bits = viterbi_mother_erasures(depuncture_erasures(hdr_coded_rx, HDR_RATE))

    if len(hdr_bits) < HDR_INFO_BITS:
        hdr_bits += [0] * (HDR_INFO_BITS - len(hdr_bits))

    hdr_bits = hdr_bits[:HDR_INFO_BITS]

    phr_rx = hdr_bits[:32]
    got_hcs = bits16_lsb_to_u16(hdr_bits[32:48])
    calc_hcs = crc16_hcs_lsb(phr_rx, init=0xFFFF)

    parsed = parse_phr_32_lsb(phr_rx)

    if (parsed is None) or (got_hcs != calc_hcs):
        return False

    mcs_rx, length = parsed
    rate = MCS_TO_RATE.get(mcs_rx, None)

    if rate is None:
        return False

    if not (0 < length <= 5000):
        return False

    psdu_info_bits = length * 8
    psdu_coded_len = coded_bits_len_for_info(psdu_info_bits, rate)

    ptr2 = ptr + hdr_coded_len
    psdu_coded_rx = rx[ptr2:ptr2 + psdu_coded_len]

    psdu_bits = viterbi_mother_erasures(depuncture_erasures(psdu_coded_rx, rate))

    if len(psdu_bits) < psdu_info_bits:
        psdu_bits += [0] * (psdu_info_bits - len(psdu_bits))

    psdu_bits = psdu_bits[:psdu_info_bits]

    msg = bits_to_bytes_lsb(psdu_bits)[:length]

    return msg == PAYLOAD_BYTES

def manual_check() -> None:
    rng = random.Random(SEED)
    ok_count = 0

    for _ in range(N_TRIALS):
        ok = run_trial(BIT_ERR_HEADER, BIT_ERR_PAYLOAD, rng)

        if ok:
            ok_count += 1
        else:
            if CRITERIO_100_OK:
                break

    verdict = (ok_count == N_TRIALS) if CRITERIO_100_OK else (ok_count > 0)

    print(f"PAYLOAD_RATE={PAYLOAD_RATE}")
    print(f"BIT_ERR_HEADER={BIT_ERR_HEADER} | BIT_ERR_PAYLOAD={BIT_ERR_PAYLOAD}")
    print(f"RESULTADO: {'OK' if verdict else 'FAIL'}")

def main():
    manual_check()

if __name__ == "__main__":
    main()