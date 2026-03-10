#!/usr/bin/env python3
"""
GPS Acquisition Live Monitor  —  Basys-3 UART
==============================================
Dos ventanas:
  Ventana 1: barras en tiempo real, 1 barra por PRN, actualización PRN a PRN.
             Eje Y = SNR proxy (prn_best_accum >> 4), permite comparar
             intensidad de señal entre satélites adquiridos.
  Ventana 2: métricas de debugging (historial, varianza, tasa detección, etc.)

Ejecutar desde terminal (NO desde Spyder):
  python gps_monitor.py --port COM3           (Windows)
  python gps_monitor.py --port /dev/ttyUSB1   (Linux)
  python gps_monitor.py                       (autodetecta puerto)
  python gps_monitor.py --prns 32             (32 satélites)
  python gps_monitor.py --demo                (sin hardware, datos simulados)

Dependencias:
  pip install pyserial matplotlib numpy

Formato UART esperado (v13 firmware):
  SAT XX: ACQUIRED  doppler=±NNNNN Hz  phase=NNNN chips  snr=NNNNN
  SAT XX: NO LOCK

Formato HEX v12 firmware (sin prefijo 0x, 'sats'):
    SAT PP: ACQ dop=SDD ph=PPP snr=SSSS
    SAT PP: NOLOCK
    TOTAL: TT sats

Etiquetas encima de cada barra: Doppler (kHz) y Code Phase (chips).
El eje Y muestra el SNR proxy (adim. = adimensional, escala interna FPGA).
"""

import argparse
import re
import sys
import threading
import time
import random
from collections import deque, defaultdict

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from matplotlib.colors import LinearSegmentedColormap

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# ──────────────────────────────────────────────────────────────
# Paleta
# ──────────────────────────────────────────────────────────────
BG        = '#080d14'
PANEL     = '#0f1923'
ACQ_CLR   = '#00e5ff'
NOLOCK    = '#162032'
SCANNING  = '#f59e0b'
GRID_CLR  = '#162032'
TEXT_CLR  = '#64748b'
TITLE_CLR = '#e2e8f0'
TOTAL_CLR = '#fbbf24'
GOOD_CLR  = '#22c55e'
WARN_CLR  = '#f59e0b'
BAD_CLR   = '#ef4444'
PORT_CLRS = {'ok': '#22c55e', 'error': '#ef4444', 'connecting': '#64748b'}

CMAP_HEAT = LinearSegmentedColormap.from_list(
    'gps', ['#0f1923', '#0e3a5f', '#00e5ff', '#fbbf24', '#ef4444'])

matplotlib.rcParams.update({
    'font.family':     'monospace',
    'axes.facecolor':  PANEL,
    'figure.facecolor': BG,
    'axes.edgecolor':  GRID_CLR,
    'axes.labelcolor': TEXT_CLR,
    'xtick.color':     TEXT_CLR,
    'ytick.color':     TEXT_CLR,
    'text.color':      TEXT_CLR,
    'grid.color':      GRID_CLR,
    'grid.linewidth':  0.5,
    'grid.linestyle':  '--',
})

# ──────────────────────────────────────────────────────────────
# Regex
# NUEVO: captura campo snr=NNNNN al final de líneas ACQUIRED
# ──────────────────────────────────────────────────────────────
RE_ACQ    = re.compile(
    r'SAT\s+(\d+):\s+ACQUIRED\s+doppler=([+-]\d+)\s+Hz\s+phase=(\d+)\s+chips'
    r'(?:\s+snr=(\d+))?'   # snr es opcional para compatibilidad con firmware anterior
)
RE_NOLOCK = re.compile(r'SAT\s+(\d+):\s+NO LOCK')
RE_TOTAL  = re.compile(r'TOTAL:\s+(\d+)\s+satellites')
# v12 firmware hex format: SAT XX: ACQ dop=SXX ph=XXX snr=XXXX  (no 0x prefix)
RE_ACQ_HEX = re.compile(
    r'SAT\s+([0-9A-Fa-f]{2}):\s+ACQ\s+dop=([+-])([0-9A-Fa-f]{2})\s+ph=([0-9A-Fa-f]{3})\s+snr=([0-9A-Fa-f]{4})'
    r'(?:\s+n=([0-9A-Fa-f]{4})\s+m=([0-9A-Fa-f]{4})\s+f=([0-9A-Fa-f]{2}))?'
)
RE_NOLOCK_HEX = re.compile(
    r'SAT\s+([0-9A-Fa-f]{2}):\s+NOLOCK(?:\s+n=([0-9A-Fa-f]{4})\s+m=([0-9A-Fa-f]{4})\s+f=([0-9A-Fa-f]{2}))?'
)
RE_TOTAL_HEX  = re.compile(r'TOTAL:\s+([0-9A-Fa-f]{2})\s+sats')
RE_HDR    = re.compile(r'=== GPS ACQUISITION RESULTS ===')
RE_FTR    = re.compile(r'^={10,}$')

HISTORY = 30   # barridos en memoria

# SNR de referencia para escalar el eje Y del gráfico live.
# Con N_INT=5 y peak_val≈500 (señal sintética) → snr≈4882.
# Con N_INT=10 y señal real débil → puede ser mucho menor.
# El eje Y se autoescala al máximo visto + 20%.
SNR_MIN_DISPLAY = 100   # mínimo eje Y para que las barras de ruido sean visibles


# ──────────────────────────────────────────────────────────────
# Estado compartido
# ──────────────────────────────────────────────────────────────
class State:
    def __init__(self, num_prns):
        self.lock     = threading.Lock()
        self.num_prns = num_prns

        self.live     = {}           # prn→{acquired,doppler,phase,snr,ts}
        self.scanning = set()
        self.in_sweep = False

        self.history  = deque(maxlen=HISTORY)
        self.sw_ts    = deque(maxlen=HISTORY)
        self.sw_dur   = deque(maxlen=HISTORY)
        self._t0      = None
        self._cur     = {}

        self.dop_hist  = defaultdict(lambda: deque(maxlen=HISTORY))
        self.ph_hist   = defaultdict(lambda: deque(maxlen=HISTORY))
        self.snr_hist  = defaultdict(lambda: deque(maxlen=HISTORY))  # NUEVO
        self.det_hist  = defaultdict(lambda: deque(maxlen=HISTORY))

        self.sweep_count = 0
        self.total_sats  = 0
        self.port_status = 'connecting'
        self.status_msg  = ''
        self.last_ts     = None

    def on_line(self, raw):
        line = raw.strip()
        if not line:
            return

        def s8_from_hex(h2):
            v = int(h2, 16)
            return v - 256 if v >= 128 else v

        if RE_HDR.match(line):
            with self.lock:
                self.in_sweep = True
                self._cur     = {}
                self._t0      = time.time()
                # Solo mostrar SCANNING en el primer barrido completo.
                # A partir del segundo, las barras muestran los valores
                # anteriores y se actualizan PRN a PRN conforme llegan.
                if self.sweep_count == 0:
                    self.scanning = set(range(1, self.num_prns + 1))
            return

        if RE_FTR.match(line) and self.in_sweep:
            with self.lock:
                self.in_sweep = False
                t_end = time.time()
                snap  = dict(self._cur)
                self.history.append(snap)
                self.sw_ts.append(t_end)
                if self._t0:
                    self.sw_dur.append(t_end - self._t0)
                self.sweep_count += 1
                self.scanning = set()
                for p in range(1, self.num_prns + 1):
                    d = snap.get(p)
                    if d and d['acquired']:
                        self.dop_hist[p].append(d['doppler'])
                        self.ph_hist[p].append(d['phase'])
                        self.snr_hist[p].append(d['snr'])   # NUEVO
                        self.det_hist[p].append(1)
                    else:
                        self.dop_hist[p].append(None)
                        self.ph_hist[p].append(None)
                        self.snr_hist[p].append(None)        # NUEVO
                        self.det_hist[p].append(0)
            return

        m = RE_ACQ.match(line)
        if m:
            prn   = int(m.group(1))
            dop   = int(m.group(2))
            phase = int(m.group(3))
            snr   = int(m.group(4)) if m.group(4) is not None else 0  # NUEVO
            with self.lock:
                self.last_ts = time.time()
                d = {'acquired': True, 'doppler': dop,
                     'phase': phase, 'snr': snr, 'ts': time.time()}
                self.live[prn] = d
                self._cur[prn] = d
                self.scanning.discard(prn)
            return

        m = RE_ACQ_HEX.match(line)
        if m:
            prn     = int(m.group(1), 16)
            # v12/v13 hex format:
            # group(2)=sign(+/-), group(3)=magnitude hex, group(4)=phase, group(5)=snr
            # group(6..8)=optional n/m/f debug telemetry
            dop_mag = int(m.group(3), 16)
            dop_bin = dop_mag if m.group(2) == '+' else -dop_mag
            dop     = dop_bin * 320
            phase   = int(m.group(4), 16)
            snr     = int(m.group(5), 16)
            noise   = int(m.group(6), 16) if m.group(6) is not None else 0
            margin  = int(m.group(7), 16) if m.group(7) is not None else 0
            flags   = int(m.group(8), 16) if m.group(8) is not None else 0
            with self.lock:
                self.last_ts = time.time()
                d = {'acquired': True, 'doppler': dop,
                     'phase': phase, 'snr': snr,
                     'noise': noise, 'margin': margin, 'flags': flags,
                     'ts': time.time()}
                self.live[prn] = d
                self._cur[prn] = d
                self.scanning.discard(prn)
            return

        m = RE_NOLOCK.match(line)
        if m:
            prn = int(m.group(1))
            with self.lock:
                self.last_ts = time.time()
                d = {'acquired': False, 'doppler': 0,
                     'phase': 0, 'snr': 0, 'ts': time.time()}
                self.live[prn] = d
                self._cur[prn] = d
                self.scanning.discard(prn)
            return

        m = RE_NOLOCK_HEX.match(line)
        if m:
            prn = int(m.group(1), 16)
            noise = int(m.group(2), 16) if m.group(2) is not None else 0
            margin = int(m.group(3), 16) if m.group(3) is not None else 0
            flags = int(m.group(4), 16) if m.group(4) is not None else 0
            with self.lock:
                self.last_ts = time.time()
                d = {'acquired': False, 'doppler': 0,
                     'phase': 0, 'snr': 0,
                     'noise': noise, 'margin': margin, 'flags': flags,
                     'ts': time.time()}
                self.live[prn] = d
                self._cur[prn] = d
                self.scanning.discard(prn)
            return

        m = RE_TOTAL.match(line)
        if m:
            with self.lock:
                self.total_sats = int(m.group(1))
            return

        m = RE_TOTAL_HEX.match(line)
        if m:
            with self.lock:
                self.total_sats = int(m.group(1), 16)


# ──────────────────────────────────────────────────────────────
# Hilo serie
# ──────────────────────────────────────────────────────────────
def serial_reader(port, baud, state, stop):
    while not stop.is_set():
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                state.port_status = 'ok'
                state.status_msg  = f'{port}  {baud} baud'
                while not stop.is_set():
                    raw = ser.readline().decode('ascii', errors='replace')
                    if raw:
                        state.on_line(raw)
        except Exception as e:
            state.port_status = 'error'
            state.status_msg  = str(e)[:60]
            time.sleep(2)


# ──────────────────────────────────────────────────────────────
# Demo sin hardware
# SNR simulado: satélites adquiridos tienen valores distintos
# para verificar que el eje Y discrimina bien entre ellos.
# ──────────────────────────────────────────────────────────────
def demo_thread(state, stop, num_prns):
    # PRN→(doppler_Hz, phase_chips, snr_base)
    # snr distintos para mostrar diferencia de potencia entre sats
    # PRN → (doppler_bin, phase_chips, snr_base)
    # doppler_bin * 320 Hz = Doppler Hz; bin+8=+2560 Hz, bin-12=-3840 Hz, bin+16=+5120 Hz
    SAT = {
        1: (+8,    100, 0x1D40),   # señal fuerte
        2: (-12,   300, 0x0FB3),   # señal media
        3: (+16,   600, 0x0CD0),   # señal débil (pero por encima del umbral)
    }
    state.port_status = 'ok'
    state.status_msg  = 'DEMO (sin hardware) — formato v12 HEX'
    while not stop.is_set():
        state.on_line("=== GPS ACQUISITION RESULTS ===")
        for p in range(1, num_prns + 1):
            time.sleep(0.15)
            if p in SAT:
                dbin, ph, snr_base = SAT[p]
                dbin += random.randint(-1, 1)
                ph   += random.randint(-1, 1)
                snr   = snr_base + random.randint(-snr_base // 10,
                                                   snr_base // 10)
                snr   = max(0, snr) & 0xFFFF
                sign  = '+' if dbin >= 0 else '-'
                state.on_line(
                    f"SAT {p:02X}: ACQ dop={sign}{abs(dbin):02X} "
                    f"ph={ph:03X} snr={snr:04X}")
            else:
                state.on_line(f"SAT {p:02X}: NOLOCK")
        state.on_line(f"TOTAL: {len(SAT):02X} sats")
        state.on_line("=" * 32)
        time.sleep(0.3)


def autodetect():
    if not HAS_SERIAL:
        return None
    for p in serial.tools.list_ports.comports():
        if any(k in (p.description or '').lower()
               for k in ('usb', 'uart', 'cp210', 'ch340', 'ftdi',
                         'basys', 'digilent', 'prolific')):
            return p.device
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return ports[0] if ports else None


# ──────────────────────────────────────────────────────────────
# VENTANA 1 — Live bars  (eje Y = SNR proxy)
# ──────────────────────────────────────────────────────────────
def build_win1(num_prns):
    fig = plt.figure(figsize=(max(14, num_prns * 0.6), 7),
                     facecolor=BG, num='GPS Live')
    fig.canvas.manager.set_window_title('GPS Acquisition — Live')
    gs  = gridspec.GridSpec(2, 1, height_ratios=[8, 1],
                            hspace=0.06, left=0.05, right=0.98,
                            top=0.92, bottom=0.04)
    ax_b = fig.add_subplot(gs[0])
    ax_s = fig.add_subplot(gs[1])
    ax_s.axis('off')
    fig.text(0.5, 0.965, 'GPS ACQUISITION  —  LIVE',
             ha='center', color=TITLE_CLR, fontsize=14,
             fontweight='bold')
    return fig, ax_b, ax_s


def update_win1(frame, ax_b, ax_s, state, num_prns):
    with state.lock:
        live     = dict(state.live)
        scanning = set(state.scanning)
        sc       = state.sweep_count
        tot      = state.total_sats
        pst      = state.port_status
        pmsg     = state.status_msg
        lts      = state.last_ts
        durs     = list(state.sw_dur)

    # Calcular rango Y dinámico basado en SNR máximo visto
    max_snr = max(
        (r['snr'] for r in live.values() if r.get('acquired') and r.get('snr', 0) > 0),
        default=SNR_MIN_DISPLAY
    )
    y_max = max(SNR_MIN_DISPLAY, int(max_snr * 1.25))   # margen 25% arriba

    ax_b.cla()
    ax_b.set_facecolor(PANEL)
    ax_b.set_xlim(-0.5, num_prns - 0.5)
    ax_b.set_ylim(0, y_max)
    ax_b.set_ylabel('SNR proxy  (adim.)', fontsize=9)
    ax_b.yaxis.grid(True)
    ax_b.set_axisbelow(True)
    for sp in ax_b.spines.values():
        sp.set_color(GRID_CLR)

    xs = list(range(num_prns))
    heights, colors = [], []
    for i in range(num_prns):
        prn = i + 1
        r   = live.get(prn)
        if prn in scanning:
            heights.append(y_max * 0.5)   # barra a la mitad mientras escanea
            colors.append(SCANNING)
        elif r and r['acquired']:
            heights.append(r.get('snr', 0))
            colors.append(ACQ_CLR)
        else:
            heights.append(y_max * 0.015)  # barra mínima visible para NO LOCK
            colors.append(NOLOCK)

    bars = ax_b.bar(xs, heights, color=colors, width=0.75,
                    edgecolor='none', zorder=3)

    for i, bar in enumerate(bars):
        prn = i + 1
        r   = live.get(prn)
        if prn in scanning:
            ax_b.text(bar.get_x() + bar.get_width()/2,
                      bar.get_height() + y_max * 0.03, '···',
                      ha='center', color=SCANNING, fontsize=9)
        elif r and r['acquired']:
            sign = '+' if r['doppler'] >= 0 else ''
            # Muestra Doppler y Code Phase encima de la barra
            lbl  = f"{sign}{r['doppler']/1000:.1f}k\n{r.get('phase', 0)} chips"
            ax_b.text(bar.get_x() + bar.get_width()/2,
                      bar.get_height() + y_max * 0.02, lbl,
                      ha='center', va='bottom', color=ACQ_CLR,
                      fontsize=7, fontweight='bold')
        else:
            ax_b.text(bar.get_x() + bar.get_width()/2,
                      y_max * 0.03, '—',
                      ha='center', color='#253040', fontsize=9)

    ax_b.set_xticks(xs)
    ax_b.set_xticklabels([f'PRN\n{i+1:02d}' for i in range(num_prns)], fontsize=7)

    # Status bar
    ax_s.cla()
    ax_s.set_facecolor(PANEL)
    ax_s.set_xlim(0, 1)
    ax_s.set_ylim(0, 1)
    ax_s.axis('off')
    ax_s.axhline(1.0, color=GRID_CLR, lw=0.8)

    ax_s.text(0.01, 0.5,
              f'● {pmsg}',
              color=PORT_CLRS.get(pst, TEXT_CLR), fontsize=8, va='center')

    elapsed = f'{time.time()-lts:.1f}s ago' if lts else 'waiting…'
    avg_dur = f'{np.mean(durs):.1f}s' if durs else '?'
    ax_s.text(0.30, 0.5,
              f'Sweep #{sc}   último PRN: {elapsed}   avg sweep: {avg_dur}',
              color=TEXT_CLR, fontsize=8, va='center')

    ax_s.text(0.73, 0.5,
              f'ACQUIRED: {tot:02d} / {num_prns:02d}',
              color=GOOD_CLR if tot > 0 else TEXT_CLR,
              fontsize=11, fontweight='bold', va='center', ha='center')

    leg = [mpatches.Patch(color=ACQ_CLR,  label='ACQUIRED'),
           mpatches.Patch(color=SCANNING, label='SCANNING'),
           mpatches.Patch(color=NOLOCK,   label='NO LOCK')]
    ax_s.legend(handles=leg, loc='center right',
                facecolor=PANEL, edgecolor=GRID_CLR,
                labelcolor=TEXT_CLR, fontsize=7, framealpha=1, ncol=3)


# ──────────────────────────────────────────────────────────────
# VENTANA 2 — Métricas
# ──────────────────────────────────────────────────────────────
def _ax(fig, gs_item):
    ax = fig.add_subplot(gs_item)
    ax.set_facecolor(PANEL)
    for sp in ax.spines.values():
        sp.set_color(GRID_CLR)
    return ax


def update_win2(frame, fig, state, num_prns):
    with state.lock:
        history = list(state.history)
        sw_dur  = list(state.sw_dur)
        sc      = state.sweep_count
        dop_h   = {p: list(state.dop_hist[p])  for p in range(1, num_prns+1)}
        ph_h    = {p: list(state.ph_hist[p])   for p in range(1, num_prns+1)}
        snr_h   = {p: list(state.snr_hist[p])  for p in range(1, num_prns+1)}  # NUEVO
        det_h   = {p: list(state.det_hist[p])  for p in range(1, num_prns+1)}

    fig.clear()
    fig.text(0.5, 0.978,
             'GPS ACQUISITION  —  MÉTRICAS DE DEBUGGING',
             ha='center', color=TITLE_CLR, fontsize=13, fontweight='bold')

    gs = gridspec.GridSpec(3, 3, figure=fig,
                           left=0.06, right=0.98,
                           top=0.94, bottom=0.06,
                           hspace=0.58, wspace=0.38)

    prns = list(range(1, num_prns + 1))

    # ── [0,0]  Tasa de detección ────────────────────────────
    ax = _ax(fig, gs[0, 0])
    rates    = [sum(det_h[p]) / max(len(det_h[p]), 1) for p in prns]
    bar_clrs = [GOOD_CLR if r > 0.8 else WARN_CLR if r > 0.3 else BAD_CLR
                for r in rates]
    ax.bar(prns, rates, color=bar_clrs, width=0.7, edgecolor='none')
    ax.set_xlim(0.5, num_prns + 0.5)
    ax.set_ylim(0, 1.15)
    ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
    ax.yaxis.grid(True)
    ax.set_axisbelow(True)
    ax.axhline(0.8, color=GOOD_CLR, lw=0.8, ls='--', alpha=0.5)
    ax.set_xlabel('PRN', fontsize=8)
    ax.set_title('Tasa de detección', fontsize=9, color=TITLE_CLR, pad=4)
    for i, r in enumerate(rates):
        if r > 0.05:
            ax.text(prns[i], r + 0.04, f'{r:.0%}',
                    ha='center', fontsize=6, color=TEXT_CLR)

    # ── [0,1]  Varianza Doppler σ ───────────────────────────
    ax = _ax(fig, gs[0, 1])
    dop_std = []
    for p in prns:
        v = [d for d in dop_h[p] if d is not None]
        dop_std.append(np.std(v) if len(v) > 1 else 0)
    clrs = [GOOD_CLR if s < 320 else WARN_CLR if s < 960 else BAD_CLR
            for s in dop_std]
    ax.bar(prns, dop_std, color=clrs, width=0.7, edgecolor='none')
    ax.set_xlim(0.5, num_prns + 0.5)
    ax.yaxis.grid(True)
    ax.set_axisbelow(True)
    ax.axhline(320, color=WARN_CLR, lw=0.8, ls='--', alpha=0.6,
               label='1 bin = 320 Hz')
    ax.legend(fontsize=7, facecolor=PANEL, edgecolor=GRID_CLR,
              labelcolor=TEXT_CLR)
    ax.set_xlabel('PRN', fontsize=8)
    ax.set_ylabel('Hz', fontsize=8)
    ax.set_title('Varianza Doppler  σ (Hz)', fontsize=9,
                 color=TITLE_CLR, pad=4)

    # ── [0,2]  Varianza Phase σ ─────────────────────────────
    ax = _ax(fig, gs[0, 2])
    ph_std = []
    for p in prns:
        v = [x for x in ph_h[p] if x is not None]
        ph_std.append(np.std(v) if len(v) > 1 else 0)
    clrs = [GOOD_CLR if s < 2 else WARN_CLR if s < 5 else BAD_CLR
            for s in ph_std]
    ax.bar(prns, ph_std, color=clrs, width=0.7, edgecolor='none')
    ax.set_xlim(0.5, num_prns + 0.5)
    ax.yaxis.grid(True)
    ax.set_axisbelow(True)
    ax.axhline(2, color=WARN_CLR, lw=0.8, ls='--', alpha=0.6,
               label='±2 chips')
    ax.legend(fontsize=7, facecolor=PANEL, edgecolor=GRID_CLR,
              labelcolor=TEXT_CLR)
    ax.set_xlabel('PRN', fontsize=8)
    ax.set_ylabel('chips', fontsize=8)
    ax.set_title('Varianza Phase  σ (chips)', fontsize=9,
                 color=TITLE_CLR, pad=4)

    # ── [1,0]  Historial Doppler ────────────────────────────
    ax = _ax(fig, gs[1, 0])
    det_prns = [p for p in prns if any(v is not None for v in dop_h[p])]
    pal = plt.cm.plasma(np.linspace(0.15, 0.9, max(len(det_prns), 1)))
    for i, p in enumerate(det_prns):
        xs = [j for j, v in enumerate(dop_h[p]) if v is not None]
        ys = [v for v in dop_h[p] if v is not None]
        if xs:
            ax.plot(xs, ys, '.-', color=pal[i], lw=1.2, ms=4,
                    label=f'PRN{p:02d}')
    ax.yaxis.grid(True)
    ax.set_axisbelow(True)
    ax.set_xlabel('Sweep #', fontsize=8)
    ax.set_ylabel('Hz', fontsize=8)
    ax.set_title('Historial Doppler', fontsize=9, color=TITLE_CLR, pad=4)
    if det_prns:
        ax.legend(fontsize=7, facecolor=PANEL, edgecolor=GRID_CLR,
                  labelcolor=TEXT_CLR, ncol=2)

    # ── [1,1]  Historial SNR  (NUEVO — antes era Historial Phase)
    ax = _ax(fig, gs[1, 1])
    for i, p in enumerate(det_prns):
        xs = [j for j, v in enumerate(snr_h[p]) if v is not None]
        ys = [v for v in snr_h[p] if v is not None]
        if xs:
            ax.plot(xs, ys, '.-', color=pal[i], lw=1.2, ms=4,
                    label=f'PRN{p:02d}')
    ax.yaxis.grid(True)
    ax.set_axisbelow(True)
    ax.set_xlabel('Sweep #', fontsize=8)
    ax.set_ylabel('SNR (adim.)', fontsize=8)
    ax.set_title('Historial SNR  (comparar potencia de señal)',
                 fontsize=9, color=TITLE_CLR, pad=4)
    if det_prns:
        ax.legend(fontsize=7, facecolor=PANEL, edgecolor=GRID_CLR,
                  labelcolor=TEXT_CLR, ncol=2)

    # ── [1,2]  Historial Phase ──────────────────────────────
    ax = _ax(fig, gs[1, 2])
    for i, p in enumerate(det_prns):
        xs = [j for j, v in enumerate(ph_h[p]) if v is not None]
        ys = [v for v in ph_h[p] if v is not None]
        if xs:
            ax.plot(xs, ys, '.-', color=pal[i], lw=1.2, ms=4,
                    label=f'PRN{p:02d}')
    ax.yaxis.grid(True)
    ax.set_axisbelow(True)
    ax.set_xlabel('Sweep #', fontsize=8)
    ax.set_ylabel('chips', fontsize=8)
    ax.set_title('Historial Phase', fontsize=9, color=TITLE_CLR, pad=4)
    if det_prns:
        ax.legend(fontsize=7, facecolor=PANEL, edgecolor=GRID_CLR,
                  labelcolor=TEXT_CLR, ncol=2)

    # ── [2, 0:2]  Mapa de calor: detección × barrido ────────
    ax = _ax(fig, gs[2, 0:2])
    n_sw = len(history)
    if n_sw > 0:
        mat = np.zeros((num_prns, n_sw))
        for si, snap in enumerate(history):
            for pi, p in enumerate(prns):
                d = snap.get(p)
                mat[pi, si] = 1.0 if (d and d['acquired']) else 0.0
        im = ax.imshow(mat, aspect='auto', cmap=CMAP_HEAT,
                       vmin=0, vmax=1, interpolation='nearest',
                       extent=[-0.5, n_sw - 0.5, num_prns + 0.5, 0.5])
        ax.set_yticks(prns)
        ax.set_yticklabels([f'PRN{p:02d}' for p in prns], fontsize=6)
        ax.set_xlabel('Sweep #', fontsize=8)
        ax.set_title(
            'Mapa de detección  '
            '(rojo = ACQUIRED,  azul oscuro = NO LOCK)',
            fontsize=9, color=TITLE_CLR, pad=4)
        fig.colorbar(im, ax=ax, shrink=0.8)
    else:
        ax.text(0.5, 0.5, 'Esperando datos…',
                ha='center', va='center', color=TEXT_CLR, fontsize=10)
        ax.set_title('Mapa de detección', fontsize=9,
                     color=TITLE_CLR, pad=4)

    # ── [2, 2]  Tabla resumen ────────────────────────────────
    ax = _ax(fig, gs[2, 2])
    ax.axis('off')
    ax.set_title('Resumen estadístico', fontsize=9, color=TITLE_CLR, pad=4)

    rows = []
    for p in prns:
        dv  = [d for d in dop_h[p]  if d is not None]
        sv  = [v for v in snr_h[p]  if v is not None]   # NUEVO
        det = det_h[p]
        if not any(det):
            continue
        rate  = sum(det) / max(len(det), 1)
        dop_mean_val = np.mean(dv) if dv else 0
        dop_std_val  = np.std(dv)  if len(dv) > 1 else 0
        dop_m = f'{dop_mean_val:+.0f}' if dv else '—'
        if dop_std_val > 0:
            dop_cell = f'{dop_m}±{dop_std_val:.0f} Hz'
        else:
            dop_cell = f'{dop_m} Hz'
        # SNR medio (NUEVO — antes era phase)
        snr_mean = f'{np.mean(sv):.0f}' if sv else '—'
        rows.append([f'PRN{p:02d}',
                     f'{rate:.0%}',
                     dop_cell,
                     snr_mean])   # NUEVO: SNR en vez de phase en tabla

    if rows:
        tbl = ax.table(cellText=rows,
                       colLabels=['PRN', 'Det.', 'Doppler', 'SNR avg'],
                       loc='center', cellLoc='center')
        tbl.auto_set_font_size(False)
        tbl.set_fontsize(7.5)
        tbl.scale(1, 1.45)
        for (r, c), cell in tbl.get_celld().items():
            cell.set_facecolor(PANEL if r > 0 else '#162032')
            cell.set_edgecolor(GRID_CLR)
            cell.set_text_props(color=ACQ_CLR if r > 0 else TITLE_CLR)
    else:
        ax.text(0.5, 0.5, 'Esperando datos…',
                ha='center', va='center', color=TEXT_CLR, fontsize=9)

    # ── Duración de barridos (movido a la parte inferior del layout)
    # Nota: en la grilla 3×3 ocupamos [2,2], la duración queda en
    # la tabla. Si se quiere recuperar el gráfico de duración,
    # ampliar a 4 filas o reemplazar la tabla.


# ──────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description='GPS Acquisition Live Monitor — Basys-3')
    ap.add_argument('--port',      '-p', default=None,
                    help='Puerto serie (COM3 o /dev/ttyUSB1)')
    ap.add_argument('--baud',      '-b', type=int, default=115200)
    ap.add_argument('--prns',      '-n', type=int, default=32,
                    help='Número de PRNs en la FPGA')
    ap.add_argument('--interval',  '-i', type=int, default=400,
                    help='Refresco ventana live (ms)')
    ap.add_argument('--interval2',       type=int, default=1500,
                    help='Refresco ventana métricas (ms)')
    ap.add_argument('--demo',      action='store_true',
                    help='Datos simulados sin hardware')
    args = ap.parse_args()

    state = State(args.prns)
    stop  = threading.Event()

    if args.demo:
        print('Modo DEMO — sin puerto serie.')
        t = threading.Thread(target=demo_thread,
                             args=(state, stop, args.prns), daemon=True)
    else:
        if not HAS_SERIAL:
            print('ERROR: instala pyserial:  pip install pyserial')
            sys.exit(1)
        port = args.port or autodetect()
        if not port:
            print('ERROR: no se encontró puerto serie.')
            print('Especifica con --port COM3  o  --port /dev/ttyUSB1')
            sys.exit(1)
        print(f'Conectando a {port}  @ {args.baud} baud')
        t = threading.Thread(target=serial_reader,
                             args=(port, args.baud, state, stop), daemon=True)
    t.start()

    # Ventana 1
    fig1, ax_b, ax_s = build_win1(args.prns)
    ani1 = FuncAnimation(fig1, update_win1,
                         fargs=(ax_b, ax_s, state, args.prns),
                         interval=args.interval, cache_frame_data=False)

    # Ventana 2
    fig2 = plt.figure(figsize=(16, 10), facecolor=BG, num='GPS Metrics')
    fig2.canvas.manager.set_window_title('GPS Acquisition — Métricas')
    ani2 = FuncAnimation(fig2, update_win2,
                         fargs=(fig2, state, args.prns),
                         interval=args.interval2, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        print('Monitor cerrado.')


if __name__ == '__main__':
    main()