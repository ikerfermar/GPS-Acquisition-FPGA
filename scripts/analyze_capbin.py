#!/usr/bin/env python3
"""
analyze_capbin.py — GPS L1 C/A dual acquisition analysis
=========================================================

Decodes a .capbin file and runs two parallel acquisition algorithms:

  Method 1  [FFT-HR]  : Parallel code-phase search at full 16.368 Msps rate.
                        FFT size = 16 384 (next power-of-2 above 16 368).
                        Code-phase resolution: 1/16 chip (~97 ns).
                        Doppler search: ±64 bins × 320 Hz/bin centred at
                        bias +64 bins (+20.48 kHz), same grid as the FPGA.
                        Non-coherent: |IFFT| accumulated over coherent_ms-ms
                        blocks (default 4 ms coherent, 5 epochs = 20 ms).

  Method 2  [RTL-EQ]  : Exact replica of the FPGA fft_controller pipeline:
                          • mix with IF+Doppler NCO at 16.368 MHz
                          • decimate ×16  →  1 023 chip-rate samples
                          • zero-pad to 1 024
                          • 1 024-point FFT circular correlation
                          • coherent accumulation over 4 frames (4 ms)
                          • non-coherent accumulation over 5 epochs (20 ms)
                        Code-phase resolution: 1 chip (~977 ns).

Signal format (.capbin):
  4 samples / byte, LSB-first.
  Each 2-bit sample — bit[1]=I1 (sign), bit[0]=I0 (magnitude):
    00 → +1,  01 → +3,  10 → −1,  11 → −3.

A single figure (one window) shows:
  • Top panel    – Method 1 [FFT-HR] SNR bar chart (one bar per PRN)
  • Middle panel – Method 2 [RTL-EQ] SNR bar chart (identical layout)
  • Bottom panel – Side-by-side comparison table (acquired PRNs only)

Usage:
  python scripts/analyze_capbin.py
  python scripts/analyze_capbin.py captures/skydel_normal_20260420_115052_1.capbin
  python scripts/analyze_capbin.py captures/diag_real_capture.capbin --prns 1,6,8,12
  python scripts/analyze_capbin.py captures/foo.capbin --threshold 3.5 --n-int 5

Dependencies: numpy, matplotlib  (no serial / FPGA hardware needed)
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import matplotlib
if '--no-show' in sys.argv or '--sweep' in sys.argv:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ─────────────────────────────────────────────────────────────────────────────
# System parameters  (mirror of gps_config_pkg.vhd)
# ─────────────────────────────────────────────────────────────────────────────
FS             = 16_368_000.0   # Sample rate (Hz)
IF_FREQ        = 4_092_000.0    # IF centre frequency (Hz)
CHIP_RATE      = 1_023_000.0    # GPS C/A chip rate (Hz)
N_CHIPS        = 1023           # Chips per 1 ms epoch
DECIMATION     = 16             # FS / chip_rate = 16
N_SAMP_PER_MS  = 16_368         # Samples per 1 ms frame
N_FRAMES_CAP   = 20             # Number of 1 ms frames in a standard capbin
BYTES_PER_FRAME = N_SAMP_PER_MS // 4   # 4 092 bytes (2 bits/sample)
TOTAL_BYTES    = N_FRAMES_CAP * BYTES_PER_FRAME   # 81 840 bytes

# Expected satellites for sensitivity scoring
EXPECTED_PRNS     = frozenset({6, 8, 16, 18, 23, 27})   # above horizon
NEAR_HORIZON_PRNS = frozenset({10, 12, 20, 26})          # marginal

# FPGA Doppler search parameters
DOP_BIN_HZ      = 320.0   # Hz per Doppler bin    (CFG_COARSE_STEP × 160)
DOP_BIAS_BINS   = 0       # Cold-start bias bins
DOP_RANGE_BINS  = 124     # ± search range         (CFG_BIN_RANGE)

# FFT size for Method 1
NFFT_HR = 1 << (N_SAMP_PER_MS - 1).bit_length()   # 16 384
# FFT size for Method 2 (matches FPGA fft_controller)
NFFT_RTL = 1024

# ─────────────────────────────────────────────────────────────────────────────
# Paths
# ─────────────────────────────────────────────────────────────────────────────
_SCRIPT_DIR = Path(__file__).resolve().parent
_REPO_ROOT  = _SCRIPT_DIR.parent
DEFAULT_CAP = _REPO_ROOT / 'captures' / 'skydel_normal_20260420_115052_1.capbin'


def resolve_capbin_path(user_arg: Optional[Path]) -> Path:
    """
    Resolve capture path with robust defaults.

    Priority:
      1) Explicit positional argument.
      2) DEFAULT_CAP if it exists.
      3) First *.capbin under captures/ (alphabetical).
    """
    if user_arg is not None:
        capbin_path = Path(user_arg)
        if not capbin_path.is_absolute():
            capbin_path = _REPO_ROOT / capbin_path
        return capbin_path

    if DEFAULT_CAP.exists():
        return DEFAULT_CAP

    captures_dir = _REPO_ROOT / 'captures'
    if captures_dir.exists():
        capbins = sorted(captures_dir.glob('*.capbin'))
        if capbins:
            return capbins[0]

    return DEFAULT_CAP

# ─────────────────────────────────────────────────────────────────────────────
# Colour palette  (consistent with gps_monitor.py)
# ─────────────────────────────────────────────────────────────────────────────
BG        = '#080d14'
PANEL     = '#0f1923'
ACQ_CLR   = '#00e5ff'
NOLOCK    = '#162032'
GOOD_CLR  = '#22c55e'
WARN_CLR  = '#f59e0b'
BAD_CLR   = '#ef4444'
GRID_CLR  = '#162032'
TEXT_CLR  = '#64748b'
TITLE_CLR = '#e2e8f0'
THR_CLR   = '#f59e0b'
M2_CLR    = '#a78bfa'   # purple accent for RTL-EQ bars

matplotlib.rcParams.update({
    'font.family':      'monospace',
    'axes.facecolor':   PANEL,
    'figure.facecolor': BG,
    'axes.edgecolor':   GRID_CLR,
    'axes.labelcolor':  TEXT_CLR,
    'xtick.color':      TEXT_CLR,
    'ytick.color':      TEXT_CLR,
    'text.color':       TEXT_CLR,
    'grid.color':       GRID_CLR,
    'grid.linewidth':   0.4,
    'grid.linestyle':   '--',
})

# ─────────────────────────────────────────────────────────────────────────────
# GPS L1 C/A code generator  (IS-GPS-200 Table 3-Ia)
# ─────────────────────────────────────────────────────────────────────────────
# G2 output tap selections per PRN (1-indexed from MSB = stage 1).
# The CA chip = G1_out XOR (G2_tap_a XOR G2_tap_b).
_CA_G2_TAPS = {
     1: (2,  6),   2: (3,  7),   3: (4,  8),   4: (5,  9),
     5: (1,  9),   6: (2, 10),   7: (1,  8),   8: (2,  9),
     9: (3, 10),  10: (2,  3),  11: (3,  4),  12: (5,  6),
    13: (6,  7),  14: (7,  8),  15: (8,  9),  16: (9, 10),
    17: (1,  4),  18: (2,  5),  19: (3,  6),  20: (4,  7),
    21: (5,  8),  22: (6,  9),  23: (1,  3),  24: (4,  6),
    25: (5,  7),  26: (6,  8),  27: (7,  9),  28: (8, 10),
    29: (1,  6),  30: (2,  7),  31: (3,  8),  32: (4,  9),
}

# Cached CA codes (computed once per PRN)
_ca_cache: dict = {}

def generate_ca(prn: int) -> np.ndarray:
    """
    Generate GPS L1 C/A gold code for PRN 1..32.

    Returns a 1023-element int8 array of ±1 values.
    G1 feedback taps: stages 3 and 10 (1-indexed).
    G2 feedback taps: stages 2, 3, 6, 8, 9, 10.
    Initial state: all-ones for both registers.
    """
    if prn in _ca_cache:
        return _ca_cache[prn]
    if prn not in _CA_G2_TAPS:
        raise ValueError(f'PRN {prn} out of range (1..32)')
    t1, t2 = _CA_G2_TAPS[prn]
    # LFSRs stored as list[10], index 0 = newest stage (stage 1), index 9 = oldest (stage 10)
    g1 = [1] * 10
    g2 = [1] * 10
    code = np.empty(N_CHIPS, dtype=np.int8)
    for i in range(N_CHIPS):
        g2_out      = g2[t1 - 1] ^ g2[t2 - 1]
        code[i]     = 1 - 2 * (g1[9] ^ g2_out)   # 0→+1, 1→−1
        fb1         = g1[2] ^ g1[9]                 # G1 feedback: stages 3, 10
        fb2         = (g2[1] ^ g2[2] ^ g2[5]        # G2 feedback: stages 2,3,6,8,9,10
                       ^ g2[7] ^ g2[8] ^ g2[9])
        g1 = [fb1] + g1[:-1]
        g2 = [fb2] + g2[:-1]
    _ca_cache[prn] = code
    return code

# ─────────────────────────────────────────────────────────────────────────────
# Capbin decoder
# ─────────────────────────────────────────────────────────────────────────────
# Sign-magnitude lookup: index = 2-bit value (I1<<1 | I0)
# I1=bit1 (sign: 0=+, 1=−),  I0=bit0 (magnitude: 0=weak, 1=strong)
#   00 → +1,  01 → +3,  10 → −1,  11 → −3
_SM_TABLE = np.array([+1.0, +3.0, -1.0, -3.0], dtype=np.float32)


def decode_capbin(path: Path, n_frames: int = N_FRAMES_CAP) -> np.ndarray:
    """
    Decode .capbin → float32 array of shape (n_frames, N_SAMP_PER_MS).

        Byte packing (historical hardware capture format):
          bits [1:0] = sample 0   (I1=bit1, I0=bit0)
          bits [3:2] = sample 1
          bits [5:4] = sample 2
          bits [7:6] = sample 3
    """
    target_bytes = n_frames * BYTES_PER_FRAME
    data = path.read_bytes()
    if len(data) < target_bytes:
        data = data + b'\x00' * (target_bytes - len(data))
    raw = np.frombuffer(data[:target_bytes], dtype=np.uint8)

    idx0 = (raw        & 0x03)   # bits [1:0]
    idx1 = ((raw >> 2) & 0x03)   # bits [3:2]
    idx2 = ((raw >> 4) & 0x03)   # bits [5:4]
    idx3 = ((raw >> 6) & 0x03)   # bits [7:6]

    samples = np.stack([idx0, idx1, idx2, idx3], axis=1).reshape(-1)
    samples = _SM_TABLE[samples]
    return samples[:n_frames * N_SAMP_PER_MS].reshape(n_frames, N_SAMP_PER_MS)

# ─────────────────────────────────────────────────────────────────────────────
# Acquisition helpers
# ─────────────────────────────────────────────────────────────────────────────

def _snr_from_corr(corr_nc: np.ndarray, valid_len: int, exclusion_radius: int = 2) -> tuple:
    """
    Compute peak SNR from a 1-D non-coherently accumulated correlation vector.
    Returns (snr, peak_val, noise_val, best_phase_idx, margin).
    SNR = peak / noise_floor  where  noise_floor = (sum − peak) / (valid_len − 1).
    Margin = peak / second_best_peak (excluding a radius around the main peak).
    """
    mag = corr_nc[:valid_len].astype(np.float32)
    best_idx = int(np.argmax(mag))
    peak  = float(mag[best_idx])
    total = float(mag.sum())
    noise = (total - peak) / max(valid_len - 1, 1)
    snr   = peak / noise if noise > 0.0 else 0.0
    
    # Calculate second peak ensuring an exclusion zone around the main peak
    mask = np.ones(valid_len, dtype=bool)
    for i in range(best_idx - exclusion_radius, best_idx + exclusion_radius + 1):
        mask[i % valid_len] = False
    
    second_peak = float(np.max(mag[mask])) if valid_len > 2*exclusion_radius else 0.0
    margin = peak / second_peak if second_peak > 0 else 0.0

    return snr, peak, noise, best_idx, margin


def _precompute_coh_ffts_hr(frames: np.ndarray,
                             dop_bins: np.ndarray,
                             coherent_ms: int,
                             n_blocks: int) -> np.ndarray:
    """
    Pre-compute coherently-accumulated signal FFTs for all Doppler bins.

    Returns complex64 array of shape (n_dop, n_blocks, NFFT_HR).
    Carrier phase is tracked continuously across frames so coherent
    integration adds constructively (IF=4092 kHz cancels exactly per ms;
    only the Doppler residual needs correction).
    """
    n_dop    = len(dop_bins)
    n_vec    = np.arange(N_SAMP_PER_MS, dtype=np.float64)
    twopi_fs = 2.0 * np.pi / FS

    out = np.zeros((n_dop, n_blocks, NFFT_HR), dtype=np.complex64)
    buf = np.zeros(NFFT_HR, dtype=np.complex64)

    for di, d_bin in enumerate(dop_bins):
        dop_hz  = (DOP_BIAS_BINS + int(d_bin)) * DOP_BIN_HZ
        carrier = np.exp(-1j * twopi_fs * (IF_FREQ + dop_hz) * n_vec).astype(np.complex64)
        # Carrier phase advance per 1 ms frame (fractional cycles only;
        # IF_FREQ × 1e-3 = 4092 exact integer → cancels mod 2π).
        frac_per_ms = (dop_hz * 1e-3) % 1.0   # fractional cycles per ms

        for blk in range(n_blocks):
            coh_acc = np.zeros(NFFT_HR, dtype=np.complex64)
            for ms in range(coherent_ms):
                fi     = (blk * coherent_ms + ms) % len(frames)
                ms_abs = blk * coherent_ms + ms
                # Phase correction: undo carrier-phase advance across frames
                phase_corr = complex(np.exp(-1j * 2.0 * np.pi * frac_per_ms * ms_abs))
                buf[:] = 0.0
                buf[:N_SAMP_PER_MS] = frames[fi] * carrier
                coh_acc += np.fft.fft(buf) * phase_corr
            out[di, blk] = coh_acc

    return out   # (n_dop, n_blocks, NFFT_HR)


def _precompute_coh_ffts_rtl(frames: np.ndarray,
                               dop_bins: np.ndarray,
                               coherent_ms: int,
                               n_blocks: int) -> np.ndarray:
    """
    Pre-compute coherently-accumulated signal FFTs for RTL-equivalent method.

    FPGA-accurate pipeline per 1 ms frame:
      1. Mix with IF+Doppler carrier at full 16.368 Msps rate.
      2. Integrate-and-dump (I&D): sum 16 samples per chip → 1023 chip-rate
         samples. This matches the FPGA doppler_mixer I&D accumulator and
         gives +12 dB SNR over naive decimation (every 16th sample).
      3. Zero-pad to NFFT_RTL = 1024.
      4. Apply carrier-phase correction and accumulate FFT over coherent_ms
         frames (continuous phase tracking as in the FPGA NCO).

    Phase correction:
      IF_FREQ = 4092 kHz → 4092 exact integer cycles/ms → cancels mod 2π.
      Doppler residual = dop_hz × 1e-3 (fractional cycles) is corrected per
      frame so coherent integration adds constructively.

    Returns complex64 array of shape (n_dop, n_blocks, NFFT_RTL).
    """
    n_dop    = len(dop_bins)
    n_vec    = np.arange(N_SAMP_PER_MS, dtype=np.float64)
    twopi_fs = 2.0 * np.pi / FS

    # Chip-boundary indices for I&D (DECIMATION=16, N_CHIPS=1023)
    iad_limit = N_CHIPS * DECIMATION   # 16368 = full ms

    out = np.zeros((n_dop, n_blocks, NFFT_RTL), dtype=np.complex64)
    buf = np.zeros(NFFT_RTL, dtype=np.complex64)

    for di, d_bin in enumerate(dop_bins):
        dop_hz  = (DOP_BIAS_BINS + int(d_bin)) * DOP_BIN_HZ
        carrier = np.exp(-1j * twopi_fs * (IF_FREQ + dop_hz) * n_vec).astype(np.complex64)
        # Carrier phase advance per 1 ms frame (fractional cycles only;
        # IF_FREQ × 1e-3 = 4092 exact integer → cancels mod 2π).
        frac_per_ms = (dop_hz * 1e-3) % 1.0   # fractional cycles per ms

        for blk in range(n_blocks):
            coh_acc = np.zeros(NFFT_RTL, dtype=np.complex64)
            for ms in range(coherent_ms):
                fi     = (blk * coherent_ms + ms) % len(frames)
                ms_abs = blk * coherent_ms + ms
                # Mix with carrier (phase from 0, same for all frames)
                mixed = frames[fi] * carrier          # (16368,) complex64
                # Integrate-and-dump: average 16 samples per chip
                iad = mixed[:iad_limit].reshape(N_CHIPS, DECIMATION).mean(axis=1)
                # Phase correction: undo carrier-phase advance across frames
                phase_corr = complex(np.exp(-1j * 2.0 * np.pi * frac_per_ms * ms_abs))
                buf[:] = 0.0
                buf[:N_CHIPS] = iad.astype(np.complex64)
                coh_acc += np.fft.fft(buf) * phase_corr
            out[di, blk] = coh_acc

    return out   # (n_dop, n_blocks, NFFT_RTL)


# ─────────────────────────────────────────────────────────────────────────────
# Per-PRN acquisition  (uses pre-computed signal FFTs)
# ─────────────────────────────────────────────────────────────────────────────

def acquire_prn_hr(coh_ffts_hr: np.ndarray,
                   dop_bins: np.ndarray,
                   prn: int) -> dict:
    """
    Method 1 [FFT-HR]: correlate pre-computed signal FFTs with CA FFT.

    coh_ffts_hr : (n_dop, n_blocks, NFFT_HR) complex64
    Returns dict with snr, dop_hz, phase_chips, peak, noise, corr_map.
    """
    ca = generate_ca(prn).astype(np.float32)
    ca_padded = np.zeros(NFFT_HR, dtype=np.float32)
    ca_padded[:N_CHIPS * DECIMATION] = np.repeat(ca, DECIMATION)
    ca_fft_conj = np.conj(np.fft.fft(ca_padded)).astype(np.complex64)  # (NFFT_HR,)

    n_dop, n_blocks, _ = coh_ffts_hr.shape

    # products[d, b, :] = coh_ffts_hr[d, b, :] * ca_fft_conj  (broadcast)
    products = coh_ffts_hr * ca_fft_conj[np.newaxis, np.newaxis, :]  # (n_dop, n_blocks, NFFT_HR)

    # Batch IFFT over last axis, then magnitude, then sum over blocks (non-coherent)
    corr_3d   = np.abs(np.fft.ifft(products, axis=-1))          # (n_dop, n_blocks, NFFT_HR)
    corr_map  = corr_3d[:, :, :N_SAMP_PER_MS].sum(axis=1)       # (n_dop, N_SAMP_PER_MS)  non-coh sum

    best_di, best_si = np.unravel_index(np.argmax(corr_map), corr_map.shape)
    # Exclusion radius of 2 chips * 16 samples/chip = 32 samples
    snr, peak, noise, _, margin = _snr_from_corr(corr_map[best_di], N_SAMP_PER_MS, exclusion_radius=32)

    dop_hz        = (DOP_BIAS_BINS + int(dop_bins[best_di])) * DOP_BIN_HZ
    phase_chips   = int(round(best_si / DECIMATION)) % N_CHIPS

    return {
        'snr':         snr,
        'peak':        peak,
        'noise':       noise,
        'dop_hz':      dop_hz,
        'dop_bin':     int(dop_bins[best_di]),
        'phase_chips': phase_chips,
        'margin':      margin,
        'phase_samp':  int(best_si),
        'corr_map':    corr_map,        # (n_dop, N_SAMP_PER_MS)
        'dop_bins':    dop_bins,
    }


def acquire_prn_rtl(coh_ffts_rtl: np.ndarray,
                    dop_bins: np.ndarray,
                    prn: int) -> dict:
    """
    Method 2 [RTL-EQ]: correlate pre-computed signal FFTs with CA FFT (1024-pt).

    coh_ffts_rtl : (n_dop, n_blocks, NFFT_RTL) complex64
    Returns dict with snr, dop_hz, phase_chips, peak, noise, corr_map.
    """
    ca = generate_ca(prn).astype(np.float32)
    ca_padded = np.zeros(NFFT_RTL, dtype=np.float32)
    ca_padded[:N_CHIPS] = ca
    ca_fft_conj = np.conj(np.fft.fft(ca_padded)).astype(np.complex64)  # (NFFT_RTL,)

    n_dop, n_blocks, _ = coh_ffts_rtl.shape

    products  = coh_ffts_rtl * ca_fft_conj[np.newaxis, np.newaxis, :]  # (n_dop, n_blocks, NFFT_RTL)
    corr_3d   = np.abs(np.fft.ifft(products, axis=-1))                  # magnitudes
    corr_map  = corr_3d[:, :, :N_CHIPS].sum(axis=1)                     # (n_dop, N_CHIPS)

    best_di, best_chip = np.unravel_index(np.argmax(corr_map), corr_map.shape)
    snr, peak, noise, _, margin = _snr_from_corr(corr_map[best_di], N_CHIPS, exclusion_radius=2)

    dop_hz      = (DOP_BIAS_BINS + int(dop_bins[best_di])) * DOP_BIN_HZ
    phase_chips = int(best_chip) % N_CHIPS

    return {
        'snr':         snr,
        'peak':        peak,
        'noise':       noise,
        'dop_hz':      dop_hz,
        'dop_bin':     int(dop_bins[best_di]),
        'phase_chips': phase_chips,
        'margin':      margin,
        'corr_map':    corr_map,   # (n_dop, N_CHIPS)
        'dop_bins':    dop_bins,
    }

# ─────────────────────────────────────────────────────────────────────────────
# Sensitivity sweep
# ─────────────────────────────────────────────────────────────────────────────

def _score_config(results_rtl: dict, threshold: float, prns: list) -> dict:
    """Score a (results, threshold) pair against expected satellites."""
    detected = {p for p in prns if results_rtl[p]['snr'] > threshold}
    tp       = detected & EXPECTED_PRNS
    near     = detected & NEAR_HORIZON_PRNS
    fp       = detected - EXPECTED_PRNS - NEAR_HORIZON_PRNS
    # TP×3 reward; near-horizon neutral (+1); FP×3 penalty
    score    = len(tp) * 3 + len(near) * 1 - len(fp) * 3
    return {
        'score':    score,
        'n_tp':     len(tp),
        'n_near':   len(near),
        'n_fp':     len(fp),
        'tp':       sorted(tp),
        'near':     sorted(near),
        'fp':       sorted(fp),
        'detected': sorted(detected),
    }


def sweep_sensitivity(frames: np.ndarray, prns: list,
                      expected_override: set = None) -> dict:
    """
    Sweep (bias_bins, coherent_ms, n_int, threshold) on Method 2 [RTL-EQ].
    Returns the best-scoring parameter dict.
    """
    global EXPECTED_PRNS, DOP_BIAS_BINS
    if expected_override:
        EXPECTED_PRNS = frozenset(expected_override)

    # (bias_bins, coherent_ms, n_int) triples to evaluate
    # bias_bins: 0 = Skydel/no-crystal-offset; 64 = hardware +13 ppm crystal
    configs = [
        ( 0, 4,  5),   # bias=0,  20 ms  — Skydel sim, standard FPGA coherence
        ( 0, 4,  3),   # bias=0,  12 ms  — coarse phase equivalent
        ( 0, 4, 10),   # bias=0,  40 ms  — extended (cycles buffer)
        ( 0, 4, 20),   # bias=0,  80 ms  — max integration with 20 frames
        ( 0, 1, 20),   # bias=0,  20 ms  — no coherent, full data
        ( 0, 1, 12),   # bias=0,  12 ms  — no coherent, partial
        (21, 4,  5),   # bias=21  (+6720 Hz, near PRN8 peak), 20 ms
        (21, 4, 10),   # bias=21, 40 ms
        (64, 4,  5),   # bias=64  (+20480 Hz, hardware crystal), 20 ms
        (64, 4, 10),   # bias=64, 40 ms
    ]
    thresholds = [1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0]

    all_rows: list = []
    _prev_bias_coh = None
    coh_rtl = None

    for bias, coh_ms, n_int in configs:
        label = f'bias={bias:+4d}  coh={coh_ms:1d}ms × n_int={n_int:2d}  ({coh_ms * n_int:3d} ms)'
        print(f'[sweep] {label}  ', end='', flush=True)
        t0 = time.perf_counter()
        # Only recompute FFTs when bias or coh_ms changes
        if (bias, coh_ms) != _prev_bias_coh:
            DOP_BIAS_BINS = bias
            dop_bins_sw = np.arange(-DOP_RANGE_BINS, DOP_RANGE_BINS + 1, dtype=np.int32)
            coh_rtl = _precompute_coh_ffts_rtl(frames, dop_bins_sw, coh_ms, n_int)
            _prev_bias_coh = (bias, coh_ms)
        else:
            # Same bias/coh but more n_int — need to recompute (blocks changed)
            coh_rtl = _precompute_coh_ffts_rtl(frames, dop_bins_sw, coh_ms, n_int)
        rtl_res: dict = {}
        for prn in prns:
            rtl_res[prn] = acquire_prn_rtl(coh_rtl, dop_bins_sw, prn)
        print(f'{time.perf_counter() - t0:.1f}s')

        for thr in thresholds:
            sc = _score_config(rtl_res, thr, prns)
            all_rows.append({
                **sc,
                'bias':      bias,
                'coh_ms':    coh_ms,
                'n_int':     n_int,
                'threshold': thr,
                'snr_all':   {p: rtl_res[p]['snr']        for p in prns},
                'dop_all':   {p: rtl_res[p]['dop_hz']     for p in prns},
                'phase_all': {p: rtl_res[p]['phase_chips'] for p in prns},
            })

    # Sort: maximise score, then TP count, then fewest FPs
    all_rows.sort(key=lambda r: (-r['score'], -r['n_tp'], r['n_fp']))

    # ── Print ranking table ────────────────────────────────────────────────
    W = 115
    print('\n' + '=' * W)
    print(' SENSITIVITY SWEEP  —  RTL-EQ Method 2')
    print(f' Expected (above horizon) : {sorted(EXPECTED_PRNS)}')
    print(f' Near-horizon (marginal)  : {sorted(NEAR_HORIZON_PRNS)}')
    print(' Score = TP×3 + near×1 − FP×3')
    print('=' * W)
    hdr = (f'  {"#":>3}  {"bias":>5}  {"coh_ms":>6}  {"n_int":>5}  {"thr":>5}  '
           f'{"score":>5}  {"TP":>2}  {"near":>4}  {"FP":>2}   detected')
    print(hdr)
    print('-' * W)
    for i, r in enumerate(all_rows[:40]):
        print(f'  {i+1:3d}  {r["bias"]:>5}  {r["coh_ms"]:>6}  {r["n_int"]:>5}  '
              f'{r["threshold"]:>5.1f}  {r["score"]:>5}  '
              f'{r["n_tp"]:>2}  {r["n_near"]:>4}  {r["n_fp"]:>2}   '
              f'{r["detected"]}')

    best = all_rows[0]

    # ── SNR breakdown for best config ──────────────────────────────────────
    print('\n' + '=' * W)
    print(f' BEST  bias={best["bias"]}  coh={best["coh_ms"]}ms × n_int={best["n_int"]}  '
          f'threshold={best["threshold"]:.1f}  '
          f'score={best["score"]}  TP={best["n_tp"]}  FP={best["n_fp"]}')
    print('=' * W)
    snr_sorted = sorted(best['snr_all'].items(), key=lambda kv: -kv[1])
    for prn, snr in snr_sorted:
        tag = ('** EXPECTED **  ' if prn in EXPECTED_PRNS
               else ('near-horizon    ' if prn in NEAR_HORIZON_PRNS
                     else '                '))
        ack = 'ACQ' if snr > best['threshold'] else '   '
        dop = best['dop_all'].get(prn, 0)
        ph  = best['phase_all'].get(prn, 0)
        print(f'  PRN {prn:02d}: SNR={snr:7.3f}  {ack}  {dop:+9.0f} Hz  '
              f'phase={ph:4d} chips  {tag}')

    # ── Translate to gps_config_pkg.vhd constants ──────────────────────────
    k_cfar     = max(1, round(best['threshold']))
    print('\n' + '=' * W)
    print(' RECOMMENDED gps_config_pkg.vhd (real profile):')
    print(f'   CFG_K_CFAR                    := {k_cfar};')
    print(f'   CFG_COHERENT_N_MS             := {best["coh_ms"]};')
    print(f'   -- CFG_N_INT (refine)         ~= {best["n_int"]};  '
          f'adjust COARSE_N_INT proportionally')
    print(f'   CFG_LOCK_SNR_MIN              := 5;  -- baseline actual')
    print(f'   CFG_LOCK_SNR_HEADROOM         := 6;  -- baseline actual')
    print(f'   CFG_REAL_DOPPLER_BIAS_BINS    := {best["bias"]};  '
          f'-- set 64 for real HW (+13ppm crystal), {best["bias"]} for Skydel replay')
    print('=' * W)

    return best


# ─────────────────────────────────────────────────────────────────────────────
# Plotting
# ─────────────────────────────────────────────────────────────────────────────

def _draw_snr_bars(ax, prns, results, threshold, acq_color, method_label):
    """Draw SNR bar chart for one acquisition method."""
    xs   = np.arange(len(prns))
    snrs = [results[p]['snr'] for p in prns]
    acq  = [s > threshold for s in snrs]
    clrs = [acq_color if a else NOLOCK for a in acq]

    y_max = max(max(snrs) * 1.35 if snrs else threshold * 3,
                threshold * 2.5, 5.0)

    bars = ax.bar(xs, snrs, color=clrs, width=0.75, edgecolor='none', zorder=3)
    ax.axhline(threshold, color=THR_CLR, lw=1.2, ls='--',
               label=f'Threshold = {threshold:.1f}', zorder=2)
    ax.set_xlim(-0.5, len(prns) - 0.5)
    ax.set_ylim(0, y_max)
    ax.set_xticks(xs)
    ax.set_xticklabels([f'P{p:02d}' for p in prns], fontsize=6.5)
    ax.set_ylabel('SNR  (peak / noise)', fontsize=8)
    ax.yaxis.grid(True)
    ax.set_axisbelow(True)
    for sp in ax.spines.values():
        sp.set_color(GRID_CLR)
    ax.legend(fontsize=8, facecolor=PANEL, edgecolor=GRID_CLR, labelcolor=TEXT_CLR,
              loc='upper right')

    n_acq = sum(acq)
    ax.text(0.01, 0.97,
            f'{method_label}   ACQUIRED: {n_acq} / {len(prns)}',
            transform=ax.transAxes, ha='left', va='top',
            fontsize=10, fontweight='bold',
            color=GOOD_CLR if n_acq > 0 else TEXT_CLR)

    for i, bar in enumerate(bars):
        p   = prns[i]
        r   = results[p]
        snr = snrs[i]
        if acq[i]:
            sign = '+' if r['dop_hz'] >= 0 else ''
            lbl  = (f"{sign}{r['dop_hz'] / 1000:.1f}k\n"
                    f"{r['phase_chips']} ch")
            ax.text(bar.get_x() + bar.get_width() / 2,
                    snr + y_max * 0.015, lbl,
                    ha='center', va='bottom', fontsize=6,
                    fontweight='bold', color=acq_color)
        else:
            ax.text(bar.get_x() + bar.get_width() / 2,
                    y_max * 0.025, '—',
                    ha='center', va='bottom', fontsize=8, color='#253040')


def plot_results(prns: list, res1: dict, res2: dict,
                 threshold: float,
                 capbin_name: str,
                 coherent_ms: int, n_int: int,
                 no_show: bool = False):
    """Build and show the single-window results figure."""

    fig = plt.figure(figsize=(max(22, len(prns) * 0.65), 15), facecolor=BG)
    fig.canvas.manager.set_window_title(
        f'GPS L1 C/A Acquisition Analysis — {capbin_name}')

    # ── Layout: 3 rows ────────────────────────────────────────────────────
    gs = gridspec.GridSpec(
        3, 1, figure=fig,
        height_ratios=[5, 5, 3],
        left=0.04, right=0.98,
        top=0.93, bottom=0.03,
        hspace=0.60)

    fig.text(
        0.5, 0.966,
        f'GPS L1 C/A Acquisition Analysis  —  {capbin_name}'
        f'   (coh={coherent_ms} ms × {n_int} epochs = {coherent_ms * n_int} ms)',
        ha='center', fontsize=13, fontweight='bold', color=TITLE_CLR)

    # ── Row 0: Method 1 [FFT-HR] ──────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0])
    ax1.set_facecolor(PANEL)
    ax1.set_title(
        f'Method 1  [FFT-HR]  —  Full-rate {NFFT_HR}-pt FFT  '
        f'(code-phase res = 1/{DECIMATION} chip ≈ 97 ns)',
        fontsize=9, color=TITLE_CLR, pad=5)
    _draw_snr_bars(ax1, prns, res1, threshold, ACQ_CLR, 'FFT-HR')

    # ── Row 1: Method 2 [RTL-EQ] ─────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1])
    ax2.set_facecolor(PANEL)
    ax2.set_title(
        f'Method 2  [RTL-EQ]  —  FPGA-equivalent {NFFT_RTL}-pt FFT  '
        f'(decimate ×{DECIMATION}, code-phase res = 1 chip ≈ 977 ns)',
        fontsize=9, color=TITLE_CLR, pad=5)
    _draw_snr_bars(ax2, prns, res2, threshold, M2_CLR, 'RTL-EQ')

    # ── Row 2: Comparison table ───────────────────────────────────────────
    ax_t = fig.add_subplot(gs[2])
    ax_t.set_facecolor(BG)
    ax_t.axis('off')
    ax_t.set_title(
        'Comparison table  —  PRNs above threshold in at least one method',
        fontsize=9, color=TITLE_CLR, pad=4)

    acq_prns = [p for p in prns
                if res1[p]['snr'] > threshold or res2[p]['snr'] > threshold]

    if acq_prns:
        rows = []
        for p in acq_prns:
            r1 = res1[p]
            r2 = res2[p]
            def _tick(r):
                return '✔ ACQ' if r['snr'] > threshold else '  —  '

            def _dop(r):
                return f"{r['dop_hz'] / 1000:+.2f} kHz"

            def _ph(r):
                return f"{r['phase_chips']:4d} chips"

            def _snr(r):
                return f"{r['snr']:5.1f}"

            rows.append([
                f'PRN {p:02d}',
                _tick(r1), _dop(r1), _ph(r1), _snr(r1),
                _tick(r2), _dop(r2), _ph(r2), _snr(r2),
            ])

        col_labels = [
            'PRN',
            'HR acq', 'HR Doppler', 'HR Phase', 'HR SNR',
            'RTL acq', 'RTL Doppler', 'RTL Phase', 'RTL SNR',
        ]
        tbl = ax_t.table(cellText=rows, colLabels=col_labels,
                         loc='center', cellLoc='center')
        tbl.auto_set_font_size(False)
        tbl.set_fontsize(8.5)
        tbl.scale(1, 1.8)
        for (r, c), cell in tbl.get_celld().items():
            cell.set_facecolor('#162032' if r == 0 else PANEL)
            cell.set_edgecolor(GRID_CLR)
            if r == 0:
                cell.set_text_props(color=TITLE_CLR, fontweight='bold')
            else:
                # highlight acquisition ticks in cyan / purple
                txt_clr = ACQ_CLR
                if c in (1, 2, 3, 4):
                    txt_clr = ACQ_CLR
                elif c in (5, 6, 7, 8):
                    txt_clr = M2_CLR
                cell.set_text_props(color=txt_clr)
    else:
        ax_t.text(0.5, 0.5,
                  'No acquisitions above threshold — try lowering --threshold',
                  ha='center', va='center', fontsize=12,
                  color=WARN_CLR, fontweight='bold')

    # ── Save + show ───────────────────────────────────────────────────────
    out_png = _REPO_ROOT / 'artifacts' / 'acquisition_analysis.png'
    out_png.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(out_png), dpi=150, bbox_inches='tight', facecolor=BG)
    print(f'[plot] Saved {out_png}')
    if not no_show:
        plt.show()

# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='GPS L1 C/A dual acquisition analysis from a .capbin capture file.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument(
        'capbin', nargs='?', type=Path, default=None,
        help='Path to .capbin file (optional: auto-detect in captures/)')
    parser.add_argument(
        '--prns', type=str, default=None,
        help='Comma-separated PRN list, e.g. "1,6,8,12,16".  Default: all 32.')
    parser.add_argument(
        '--threshold', type=float, default=3.0,
        help='SNR detection threshold (default: 3.0)')
    parser.add_argument(
        '--coherent-ms', type=int, default=4,
        help='Coherent integration length in ms (default: 4, matches FPGA)')
    parser.add_argument(
        '--n-int', type=int, default=5,
        help='Number of non-coherent accumulation epochs (default: 5 = 20 ms)')
    parser.add_argument(
        '--no-show', action='store_true',
        help='Save PNG but skip interactive window (uses Agg backend).')
    parser.add_argument(
        '--sweep', action='store_true',
        help='Sweep sensitivity configurations and recommend optimal gps_config_pkg constants.')
    parser.add_argument(
        '--dop-bias', type=int, default=None,
        help='Override Doppler bias in bins (default: 0 for Skydel sim, 64 for hardware +13ppm). '
             'Passed to sweep and single-run modes.')
    parser.add_argument(
        '--dop-range', type=int, default=None,
        help='Override Doppler ± search range in bins (default: 64).')
    parser.add_argument(
        '--expected-prns', type=str, default=None,
        help='Override expected PRNs for sweep scoring, e.g. "6,8,16,18" '
             '(default: built-in Skydel list).')
    args = parser.parse_args()

    capbin_path = resolve_capbin_path(args.capbin)
    if not capbin_path.exists():
        print(f'[ERROR] File not found: {capbin_path}', file=sys.stderr)
        print('[HINT] Provide a capture explicitly, e.g.:', file=sys.stderr)
        print('       python scripts/analyze_capbin.py captures\\my_capture.capbin', file=sys.stderr)
        print('[NOTE] .capbin files are ignored by git (see .gitignore).', file=sys.stderr)
        sys.exit(1)

    prns = list(range(1, 33))
    if args.prns:
        try:
            prns = [int(p.strip()) for p in args.prns.split(',')]
        except ValueError:
            print('[ERROR] --prns must be a comma-separated list of integers, e.g. "1,6,8"',
                  file=sys.stderr)
            sys.exit(1)
    for p in prns:
        if p not in _CA_G2_TAPS:
            print(f'[ERROR] PRN {p} out of range (1..32)', file=sys.stderr)
            sys.exit(1)

    coherent_ms = args.coherent_ms
    n_int       = args.n_int
    n_frames_needed = coherent_ms * n_int   # total frames to use (may wrap)

    expected_override = None
    if args.expected_prns:
        try:
            expected_override = {int(p.strip()) for p in args.expected_prns.split(',')}
        except ValueError:
            print('[ERROR] --expected-prns must be comma-separated integers', file=sys.stderr)
            sys.exit(1)

    # Apply Doppler overrides before anything else
    global DOP_BIAS_BINS, DOP_RANGE_BINS
    if args.dop_bias is not None:
        DOP_BIAS_BINS = args.dop_bias
    if args.dop_range is not None:
        DOP_RANGE_BINS = args.dop_range

    n_frames_in_file = max(1, (capbin_path.stat().st_size) // BYTES_PER_FRAME)
    n_frames_use = max(1, min(n_frames_in_file, N_FRAMES_CAP))

    print()
    print(f'  capbin  : {capbin_path}')
    print(f'  size    : {capbin_path.stat().st_size:,} bytes '
          f'= {n_frames_in_file} frame(s) of 1 ms')
    print(f'  PRNs    : {prns}')
    print(f'  Doppler : bias={DOP_BIAS_BINS} bins ({DOP_BIAS_BINS*DOP_BIN_HZ/1000:.1f} kHz), '
          f'range ±{DOP_RANGE_BINS} bins (±{DOP_RANGE_BINS*DOP_BIN_HZ/1000:.1f} kHz), '
          f'step {DOP_BIN_HZ:.0f} Hz  →  {2*DOP_RANGE_BINS+1} bins')
    print(f'  Coh     : {coherent_ms} ms × {n_int} epochs = {coherent_ms * n_int} ms total')
    print(f'  Frames  : using {n_frames_use} (wrapped cyclically over {n_frames_needed} needed)')
    print(f'  Thresh  : SNR > {args.threshold}')
    print()

    # ── Load capbin ────────────────────────────────────────────────────────
    t0 = time.perf_counter()
    print(f'[load] Decoding {capbin_path.name} ...', end='  ', flush=True)
    frames = decode_capbin(capbin_path, n_frames=n_frames_use)
    print(f'{frames.shape[0]} frames × {frames.shape[1]} samples  ({time.perf_counter()-t0:.2f}s)')

    # ── Doppler bin vector ─────────────────────────────────────────────────
    dop_bins = np.arange(-DOP_RANGE_BINS, DOP_RANGE_BINS + 1, dtype=np.int32)

    # ── Sweep mode (exits after printing recommendations) ─────────────────
    if args.sweep:
        sweep_sensitivity(frames, prns,
                          expected_override=expected_override)
        return

    # ── Pre-compute signal FFTs (independent of PRN) ─────────────────────
    print(f'[prep] Pre-computing carrier-mixed signal FFTs ...')
    t1 = time.perf_counter()

    print(f'       Method 1  FFT-HR  ({NFFT_HR}-pt × {len(dop_bins)} bins × {n_int} blocks) ...', end='  ', flush=True)
    coh_hr  = _precompute_coh_ffts_hr(frames, dop_bins, coherent_ms, n_int)
    print(f'{time.perf_counter()-t1:.1f}s')

    t2 = time.perf_counter()
    print(f'       Method 2  RTL-EQ  ({NFFT_RTL}-pt × {len(dop_bins)} bins × {n_int} blocks) ...', end='  ', flush=True)
    coh_rtl = _precompute_coh_ffts_rtl(frames, dop_bins, coherent_ms, n_int)
    print(f'{time.perf_counter()-t2:.1f}s')

    # ── Per-PRN acquisition ────────────────────────────────────────────────
    print(f'\n[acq]  Correlating {len(prns)} PRNs ...')
    res1: dict = {}
    res2: dict = {}

    for i, prn in enumerate(prns):
        label = f'PRN {prn:02d}'
        sys.stdout.write(f'\r       {label}  ({i+1}/{len(prns)})  ...  ')
        sys.stdout.flush()
        res1[prn] = acquire_prn_hr(coh_hr, dop_bins, prn)
        res2[prn] = acquire_prn_rtl(coh_rtl, dop_bins, prn)

    elapsed = time.perf_counter() - t0
    print(f'\r       Done in {elapsed:.1f}s                       ')

    # ── Console summary ────────────────────────────────────────────────────
    thr = args.threshold
    hdr = (f'\n  {"PRN":>5}   '
           f'{"FFT-HR SNR":>10}  {"HR Mrgn":>7}  {"HR Doppler":>12}  {"HR Phase":>8}   '
           f'{"RTL-EQ SNR":>10}  {"RTL Mrgn":>8}  {"RTL Doppler":>12}  {"RTL Phase":>8}')
    sep = '  ' + '-' * (len(hdr) - 2)
    print(hdr)
    print(sep)

    n_acq1 = 0
    n_acq2 = 0
    for p in prns:
        r1 = res1[p]
        r2 = res2[p]
        a1 = r1['snr'] > thr
        a2 = r2['snr'] > thr
        if a1:
            n_acq1 += 1
        if a2:
            n_acq2 += 1
        if a1 or a2:
            tag1 = 'ACQ' if a1 else '   '
            tag2 = 'ACQ' if a2 else '   '
            print(
                f'  PRN {p:02d}  '
                f'{r1["snr"]:8.2f} {tag1}   '
                f'{r1["margin"]:7.2f}   '
                f'{r1["dop_hz"]:+10.0f} Hz   {r1["phase_chips"]:5d} chips   '
                f'{r2["snr"]:8.2f} {tag2}   '
                f'{r2["margin"]:8.2f}   '
                f'{r2["dop_hz"]:+10.0f} Hz   {r2["phase_chips"]:5d} chips')

    print(sep)
    print(f'  ACQUIRED  FFT-HR: {n_acq1}/{len(prns)}   RTL-EQ: {n_acq2}/{len(prns)}')
    print()

    # ── Plot ───────────────────────────────────────────────────────────────
    plot_results(prns, res1, res2,
                 threshold=thr,
                 capbin_name=capbin_path.name,
                 coherent_ms=coherent_ms,
                 n_int=n_int,
                 no_show=args.no_show)


if __name__ == '__main__':
    main()
