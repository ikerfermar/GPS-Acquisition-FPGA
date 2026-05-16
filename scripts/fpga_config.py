#!/usr/bin/env python3
"""
fpga_config.py — Parse FPGA sensitivity config from gps_config_pkg.vhd
=======================================================================
Shared module for all analysis scripts. Reads the VHDL config package
and provides structured access to all acquisition/sensitivity parameters.
"""
import re
from pathlib import Path

_PKG_PATH = Path(__file__).resolve().parent.parent / 'src' / 'hdl' / 'gps_config_pkg.vhd'

_RE_INT = re.compile(
    r'constant\s+(\w+)\s*:\s*integer\s*:=\s*(-?\d+)\s*;', re.IGNORECASE)
_RE_BOOL = re.compile(
    r'constant\s+(\w+)\s*:\s*boolean\s*:=\s*(true|false)\s*;', re.IGNORECASE)

# Sensitivity-related parameters in display order
SENSITIVITY_PARAMS = [
    'CFG_COHERENT_N_MS', 'CFG_N_INT', 'CFG_COARSE_N_INT',
    'CFG_K_CFAR', 'CFG_PEAK_THRESHOLD', 'CFG_EARLY_TERM_K',
    'CFG_LOCK_SNR_MIN', 'CFG_LOCK_SNR_HEADROOM', 'CFG_MIN_MARGIN',
    'ADAPT_NOISE_SHIFT',
    'CFG_BIN_RANGE', 'CFG_INITIAL_BIN_RANGE', 'CFG_COARSE_STEP', 'CFG_REFINE_WINDOW',
    'CFG_HYST_ACQ_SWEEPS', 'CFG_HYST_REL_SWEEPS',
    'CFG_GLOBAL_TOP_N', 'CFG_SCORE_CAND_MIN', 'CFG_SCORE_PROMOTE_MIN', 'CFG_SCORE_HOLD_MIN',
    'CFG_REAL_DOPPLER_BIAS_BINS', 'CFG_PHASE_BIAS', 'CFG_PEAK_TIMEOUT_EPOCHS',
    'CFG_NUM_PRNS',
]


def parse_config(path=None):
    """Parse gps_config_pkg.vhd and return dict of all constants."""
    p = Path(path) if path else _PKG_PATH
    text = p.read_text(encoding='utf-8', errors='replace')
    cfg = {}
    for m in _RE_INT.finditer(text):
        cfg[m.group(1)] = int(m.group(2))
    for m in _RE_BOOL.finditer(text):
        cfg[m.group(1)] = m.group(2).lower() == 'true'
    return cfg


def print_config(cfg=None, title='FPGA Sensitivity Configuration (gps_config_pkg.vhd)'):
    """Print all sensitivity parameters to stdout."""
    if cfg is None:
        cfg = parse_config()
    print(f'\n{"="*65}')
    print(f'  {title}')
    print(f'{"="*65}')
    for name in SENSITIVITY_PARAMS:
        val = cfg.get(name, '?')
        short = name.replace('CFG_', '').replace('ADAPT_', '')
        print(f'  {short:30s} = {val}')
    # Derived metrics
    coh = cfg.get('CFG_COHERENT_N_MS', 4)
    nint = cfg.get('CFG_N_INT', 20)
    coarse_nint = cfg.get('CFG_COARSE_N_INT', 3)
    brange = cfg.get('CFG_BIN_RANGE', 64)
    cstep = cfg.get('CFG_COARSE_STEP', 4)
    rwin = cfg.get('CFG_REFINE_WINDOW', 2)
    nprn = cfg.get('CFG_NUM_PRNS', 32)
    coarse_bins = (2 * brange) // cstep + 1
    refine_bins = 2 * rwin + 1
    t_coarse_ms = coarse_bins * coarse_nint * coh
    t_refine_ms = refine_bins * nint * coh
    t_sweep_s = nprn * (t_coarse_ms + t_refine_ms) / 1000.0
    print(f'  {"--- Derived ---":30s}')
    print(f'  {"Coarse bins":30s} = {coarse_bins}')
    print(f'  {"Refine bins":30s} = {refine_bins}')
    print(f'  {"T_coarse/PRN":30s} = {t_coarse_ms} ms')
    print(f'  {"T_refine/PRN":30s} = {t_refine_ms} ms')
    print(f'  {"T_sweep (est)":30s} = {t_sweep_s:.1f} s')
    print(f'{"="*65}\n')


def config_text_compact(cfg=None):
    """Return compact multiline string for plot annotation."""
    if cfg is None:
        cfg = parse_config()
    lines = [
        "FPGA Config (gps_config_pkg.vhd)",
        f"COH_MS={cfg.get('CFG_COHERENT_N_MS','?')}  "
        f"N_INT={cfg.get('CFG_N_INT','?')}  "
        f"COARSE_N_INT={cfg.get('CFG_COARSE_N_INT','?')}",
        f"K_CFAR={cfg.get('CFG_K_CFAR','?')}  "
        f"PEAK_THR={cfg.get('CFG_PEAK_THRESHOLD','?')}  "
        f"EARLY_K={cfg.get('CFG_EARLY_TERM_K','?')}",
        f"SNR_MIN={cfg.get('CFG_LOCK_SNR_MIN','?')}  "
        f"HEADROOM={cfg.get('CFG_LOCK_SNR_HEADROOM','?')}  "
        f"MARGIN={cfg.get('CFG_MIN_MARGIN','?')}",
        f"ADAPT_SHIFT={cfg.get('ADAPT_NOISE_SHIFT','?')}  "
        f"HYST_ACQ={cfg.get('CFG_HYST_ACQ_SWEEPS','?')}  "
        f"HYST_REL={cfg.get('CFG_HYST_REL_SWEEPS','?')}",
        f"BIN_RANGE={cfg.get('CFG_BIN_RANGE','?')}  "
        f"COARSE_STEP={cfg.get('CFG_COARSE_STEP','?')}  "
        f"REFINE_WIN={cfg.get('CFG_REFINE_WINDOW','?')}",
        f"TOP_N={cfg.get('CFG_GLOBAL_TOP_N','?')}  "
        f"BIAS={cfg.get('CFG_REAL_DOPPLER_BIAS_BINS','?')}  "
        f"PHASE_BIAS={cfg.get('CFG_PHASE_BIAS','?')}",
        f"PEAK_TIMEOUT={cfg.get('CFG_PEAK_TIMEOUT_EPOCHS','?')}",
        f"SCORE: cand={cfg.get('CFG_SCORE_CAND_MIN','?')}  "
        f"promote={cfg.get('CFG_SCORE_PROMOTE_MIN','?')}  "
        f"hold={cfg.get('CFG_SCORE_HOLD_MIN','?')}",
    ]
    return '\n'.join(lines)


def config_text_full(cfg=None):
    """Return full multiline string with all sensitivity params."""
    if cfg is None:
        cfg = parse_config()
    lines = ['FPGA Sensitivity Config (gps_config_pkg.vhd)', '']
    for name in SENSITIVITY_PARAMS:
        val = cfg.get(name, '?')
        short = name.replace('CFG_', '').replace('ADAPT_', '')
        lines.append(f'{short:30s} = {val}')
    return '\n'.join(lines)
