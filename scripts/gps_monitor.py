#!/usr/bin/env python3
"""
GPS Acquisition Live Monitor  —  Basys-3 UART
==============================================
Tres ventanas:
  Ventana 1: barras en tiempo real, 1 barra por PRN, actualización PRN a PRN.
             Eje Y = SNR proxy (prn_best_accum >> 4), permite comparar
             intensidad de señal entre satélites adquiridos.
  Ventana 2: métricas de debugging (historial, varianza, tasa detección, etc.)
  Ventana 3: Espectro Doppler y Mapa de flags por PRN.

Ejecutar desde terminal (NO desde Spyder):
  python gps_monitor.py --port COM3           (Windows)
  python gps_monitor.py --port /dev/ttyUSB1   (Linux)
  python gps_monitor.py                       (autodetecta puerto)
  python gps_monitor.py --prns 32             (32 satélites)
  python gps_monitor.py --demo                (sin hardware, datos simulados)

Dependencias:
  pip install pyserial matplotlib numpy

Formato HEX de telemetria FPGA (sin prefijo 0x):
    SAT PP: ACQ dop=SDD ph=PPP snr=SSSS n=NNNN m=MMMM f=FF
    SAT PP: NOLOCK n=NNNN m=MMMM f=FF
    TOTAL: TT sats v=VVVVVVVV e=EEEEEEEE agc=AAAAAAAA/TTTTTTTT
    SWEEP: cycles=TTTTTTTT candidates=CC

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
from pathlib import Path

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

try:
    from fpga_config import print_config as _print_fpga_config
    from fpga_config import parse_config as _parse_fpga_config
    HAS_FPGA_CFG = True
except ImportError:
    HAS_FPGA_CFG = False

# ──────────────────────────────────────────────────────────────
# Paleta
# ──────────────────────────────────────────────────────────────
BG        = '#FFFFFF'
PANEL     = '#EFEFEF'
ACQ_CLR   = '#189C3E'
NOLOCK    = '#525668'
SCANNING  = '#0F60A7'
GRID_CLR  = '#525668'
TEXT_CLR  = '#000000'
TITLE_CLR = '#000000'
TOTAL_CLR = '#0F60A7'
GOOD_CLR  = '#189C3E'
WARN_CLR  = '#525668'
BAD_CLR   = '#0F60A7'
PORT_CLRS = {'ok': GOOD_CLR, 'error': BAD_CLR, 'connecting': NOLOCK}

CMAP_HEAT = LinearSegmentedColormap.from_list(
    'gps', [NOLOCK, SCANNING, ACQ_CLR, PANEL, BG])

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
# Regex para parsing UART HEX v12-v13
# ──────────────────────────────────────────────────────────────
# Linea de telemetria de barrido (SWEEP)
RE_SWEEP  = re.compile(
    r'SWEEP:\s+cycles=([0-9A-Fa-f]+)\s+candidates=([0-9A-Fa-f]+)'
)
# Formato ACQ: SAT XX: ACQ dop=SXX ph=XXX snr=XXXX n=XXXX m=XXXX f=FF
RE_ACQ_HEX = re.compile(
    r'SAT\s+([0-9A-Fa-f]{2}):\s+ACQ\s+dop=([+-])([0-9A-Fa-f]{2})\s+ph=([0-9A-Fa-f]{3})\s+snr=([0-9A-Fa-f]{4})'
    r'(?:\s+n=([0-9A-Fa-f]{4})\s+m=([0-9A-Fa-f]{4})\s+f=([0-9A-Fa-f]{2}))?'
)
RE_NOLOCK_HEX = re.compile(
    r'SAT\s+([0-9A-Fa-f]{2}):\s+NOLOCK(?:\s+n=([0-9A-Fa-f]{4})\s+m=([0-9A-Fa-f]{4})\s+f=([0-9A-Fa-f]{2}))?'
)
RE_TOTAL_HEX  = re.compile(
    r'TOTAL:\s+([0-9A-Fa-f]{2})\s+sats'
    r'(?:\s+v=([0-9A-Fa-f]{1,8}))?'
    r'(?:\s+e=([0-9A-Fa-f]{1,8}))?'
    r'(?:\s+agc=([0-9A-Fa-f]{1,8})/([0-9A-Fa-f]{1,8}))?'
)
RE_HDR    = re.compile(r'=== GPS ACQUISITION RESULTS ===')
RE_FTR    = re.compile(r'^={10,}$')

HISTORY = 30   # barridos en memoria

# Nombres de los 8 bits del campo 'f' de la telemétria UART (bit0=LSB)
FLAG_NAMES = ['abs_thr', 'cfar_ok', 'cfar_str', 'margin', 'sec_dom', 'snr_ok', 'raw_acq', 'lock']

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
        self.snr_hist  = defaultdict(lambda: deque(maxlen=HISTORY))
        self.det_hist  = defaultdict(lambda: deque(maxlen=HISTORY))

        self.sweep_count = 0
        self.total_sats  = 0
        self.iq_valid    = 0
        self.iq_err      = 0
        self.agc_strong  = 0
        self.agc_total   = 0
        self.sweep_time_ms = 0
        self.sweep_candidates = 0
        self.port_status = 'connecting'
        self.status_msg  = ''
        self.last_ts     = None

        # FE health history (one point per sweep)
        self.fe_agc_hist   = deque(maxlen=HISTORY)   # AGC ratio 0-1
        self.fe_trans_hist = deque(maxlen=HISTORY)   # Transition ratio e/v 0-1

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
                        self.snr_hist[p].append(d['snr'])
                        self.det_hist[p].append(1)
                    else:
                        self.dop_hist[p].append(None)
                        self.ph_hist[p].append(None)
                        self.snr_hist[p].append(None)
                        self.det_hist[p].append(0)
            return

        m = RE_ACQ_HEX.match(line)
        if m:
            prn     = int(m.group(1), 16)
        # group(2)=sign(+/-), group(3)=magnitude, group(4)=phase, group(5)=snr
        # group(6..8)=optional n/m/f telemetry
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

        m = RE_TOTAL_HEX.match(line)
        if m:
            with self.lock:
                self.total_sats = int(m.group(1), 16)
                if m.group(2) is not None:
                    self.iq_valid  = int(m.group(2), 16)
                if m.group(3) is not None:
                    self.iq_err    = int(m.group(3), 16)
                if m.group(4) is not None and m.group(5) is not None:
                    self.agc_strong = int(m.group(4), 16)
                    self.agc_total  = int(m.group(5), 16)
                    # Record FE health snapshot
                    agc_r   = self.agc_strong / self.agc_total if self.agc_total > 0 else 0.0
                    trans_r = self.iq_err / self.iq_valid if self.iq_valid > 0 else 0.0
                    self.fe_agc_hist.append(agc_r)
                    self.fe_trans_hist.append(trans_r)
            return

        m = RE_SWEEP.match(line)
        if m:
            with self.lock:
                sweep_time_cycles = int(m.group(1), 16)
                candidates_count = int(m.group(2), 16)
                # Convertir ciclos a ms (aprox. 100k ciclos = 1ms @ 100 MHz)
                sweep_time_ms = sweep_time_cycles // 100000 if sweep_time_cycles > 0 else 0
                # Guardar métricas para visualización en ventana 2
                self.sweep_time_ms = sweep_time_ms
                self.sweep_candidates = candidates_count
            return


# ──────────────────────────────────────────────────────────────
# Hilo serie
# ──────────────────────────────────────────────────────────────
def serial_reader(port, baud, state, stop, log_file=None):
    f_log = open(log_file, 'a', encoding='utf-8') if log_file else None
    try:
        while not stop.is_set():
            try:
                with serial.Serial(port, baud, timeout=1) as ser:
                    state.port_status = 'ok'
                    state.status_msg  = f'{port}  {baud} baud'
                    while not stop.is_set():
                        raw = ser.readline().decode('ascii', errors='replace')
                        if raw:
                            if f_log:
                                f_log.write(raw)
                                f_log.flush()
                            state.on_line(raw)
            except Exception as e:
                state.port_status = 'error'
                state.status_msg  = str(e)[:60]
                time.sleep(2)
    finally:
        if f_log:
            f_log.close()

def file_reader(filename, state, stop, speed=1.0):
    state.port_status = 'ok'
    state.status_msg  = f'REPLAY: {filename}'
    # Reproduce cada barrido con su duración real medida en el log (SWEEP cycles).
    # Si el log no trae esa línea, usa una pausa mínima de respaldo.
    sweep_start_t = None
    pending_sweep_s = None
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            for line in f:
                if stop.is_set():
                    break

                if RE_HDR.match(line.strip()):
                    sweep_start_t = time.perf_counter()
                    pending_sweep_s = None

                m_sweep = RE_SWEEP.search(line)
                if m_sweep:
                    cycles = int(m_sweep.group(1), 16)
                    pending_sweep_s = cycles / 100_000_000.0

                state.on_line(line)

                # Al cerrar barrido, espera hasta completar su duración real.
                if RE_FTR.match(line.strip()):
                    fallback_s = 0.8
                    target_s = (pending_sweep_s if pending_sweep_s is not None else fallback_s)
                    target_s = max(0.0, target_s / max(speed, 1e-6))
                    if sweep_start_t is not None:
                        elapsed_s = time.perf_counter() - sweep_start_t
                        remaining_s = target_s - elapsed_s
                        if remaining_s > 0:
                            time.sleep(remaining_s)
                    else:
                        time.sleep(target_s)
                else:
                    # Suavizado ligero intra-barrido para que el refresco sea legible.
                    if line.startswith("SAT") or line.startswith("TOTAL") or line.startswith("SWEEP"):
                        time.sleep(0.002)
        state.status_msg = f'REPLAY FINISHED: {filename}'
    except Exception as e:
        state.port_status = 'error'
        state.status_msg  = str(e)[:60]
    
    # Mantener el monitor abierto al terminar el archivo
    while not stop.is_set():
        time.sleep(0.5)


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
    state.status_msg  = 'DEMO (sin hardware)'
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
                    f"ph={ph:03X} snr={snr:04X} n=0014 m=0200 f=FF")
            else:
                # NOLOCK simulado
                state.on_line(f"SAT {p:02X}: NOLOCK n=0014 m=0008 f=2C")
        state.on_line(f"TOTAL: {len(SAT):02X} sats v=00500000 e=00000000 agc=001A0000/00500000")
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
        agc_s    = state.agc_strong
        agc_t    = state.agc_total
        iq_v     = state.iq_valid
        iq_e     = state.iq_err

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
                      ha='center', color=NOLOCK, fontsize=9)

    ax_b.set_xticks(xs)
    ax_b.set_xticklabels([f'PRN\n{i+1:02d}' for i in range(num_prns)], fontsize=7)

    # Status bar
    ax_s.cla()
    ax_s.set_facecolor(PANEL)
    ax_s.set_xlim(0, 1)
    ax_s.set_ylim(0, 1)
    ax_s.axis('off')

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

    # FE health indicator
    agc_ratio   = agc_s / agc_t if agc_t > 0 else -1.0
    trans_ratio = iq_e  / iq_v  if iq_v  > 0 else -1.0
    if agc_ratio < 0 and trans_ratio < 0:
        fe_txt = 'FE: —'
        fe_clr = TEXT_CLR
    else:
        # AGC target: 30–50% (MAX2769C GAINREF=170 → ~33%)
        agc_ok = 0.20 <= agc_ratio <= 0.60 if agc_ratio >= 0 else False
        # Transition ratio: synthetic=0%, real with FE active=50–80%
        tr_ok  = trans_ratio >= 0.40 if trans_ratio >= 0 else False
        agc_str  = f'{agc_ratio*100:.0f}%' if agc_ratio >= 0 else '—'
        tr_str   = f'{trans_ratio*100:.0f}%' if trans_ratio >= 0 else '—'
        fe_clr   = GOOD_CLR if (agc_ok and tr_ok) else WARN_CLR if (agc_ok or tr_ok) else BAD_CLR
        fe_txt   = f'FE AGC:{agc_str}  Trans:{tr_str}'
    ax_s.text(0.91, 0.5, fe_txt,
              color=fe_clr, fontsize=8, va='center', ha='center',
              fontweight='bold')

    leg = [mpatches.Patch(color=ACQ_CLR,  label='ACQUIRED'),
           mpatches.Patch(color=SCANNING, label='SCANNING'),
           mpatches.Patch(color=NOLOCK,   label='NO LOCK')]
    ax_s.legend(handles=leg, loc='lower right',
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
        history    = list(state.history)
        sw_dur     = list(state.sw_dur)
        sc         = state.sweep_count
        dop_h      = {p: list(state.dop_hist[p])  for p in range(1, num_prns+1)}
        ph_h       = {p: list(state.ph_hist[p])   for p in range(1, num_prns+1)}
        snr_h      = {p: list(state.snr_hist[p])  for p in range(1, num_prns+1)}
        det_h      = {p: list(state.det_hist[p])  for p in range(1, num_prns+1)}
        fe_agc_h   = list(state.fe_agc_hist)
        fe_trans_h = list(state.fe_trans_hist)
        agc_s      = state.agc_strong
        agc_t      = state.agc_total
        iq_v       = state.iq_valid
        iq_e       = state.iq_err

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

    # ── [0,2]  FE Health — AGC + Transition ratio ──────────
    ax = _ax(fig, gs[0, 2])
    agc_ratio   = agc_s / agc_t if agc_t > 0 else None
    trans_ratio = iq_e  / iq_v  if iq_v  > 0 else None

    if fe_agc_h or fe_trans_h:
        xs_fe = list(range(len(fe_agc_h)))
        if fe_agc_h:
            agc_clrs = [GOOD_CLR if 0.20 <= v <= 0.60 else WARN_CLR
                        for v in fe_agc_h]
            ax.plot(xs_fe[:len(fe_agc_h)],
                    [v * 100 for v in fe_agc_h],
                    '.-', color=ACQ_CLR, lw=1.5, ms=5, label='AGC %')
            ax.axhspan(20, 60, color=GOOD_CLR, alpha=0.08, label='AGC target 20-60%')
        if fe_trans_h:
            ax.plot(xs_fe[:len(fe_trans_h)],
                    [v * 100 for v in fe_trans_h],
                    '.-', color=WARN_CLR, lw=1.5, ms=5, label='Trans %')
            ax.axhspan(40, 85, color=WARN_CLR, alpha=0.06, label='Trans OK ≥40%')
        ax.set_ylim(0, 110)
        ax.yaxis.grid(True)
        ax.set_axisbelow(True)
        ax.set_xlabel('Sweep #', fontsize=8)
        ax.set_ylabel('%', fontsize=8)
        ax.legend(fontsize=6, facecolor=PANEL, edgecolor=GRID_CLR,
                  labelcolor=TEXT_CLR, ncol=2)
    else:
        ax.text(0.5, 0.5, 'Esperando datos TOTAL…',
                ha='center', va='center', color=TEXT_CLR, fontsize=9)
    # Annotate current values
    agc_str   = f'{agc_ratio*100:.0f}%' if agc_ratio is not None else '—'
    trans_str = f'{trans_ratio*100:.0f}%' if trans_ratio is not None else '—'
    agc_ok  = agc_ratio is not None and 0.20 <= agc_ratio <= 0.60
    tr_ok   = trans_ratio is not None and trans_ratio >= 0.40
    health  = 'OK' if (agc_ok and tr_ok) else 'PARCIAL' if (agc_ok or tr_ok) else 'SIN FE'
    hclr    = GOOD_CLR if (agc_ok and tr_ok) else WARN_CLR if (agc_ok or tr_ok) else BAD_CLR
    ax.set_title(
        f'FE Health  AGC={agc_str}  Trans={trans_str}  [{health}]',
        fontsize=8, color=hclr, pad=4)

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

    # ── [1,1]  Historial SNR ────────────────────────────────
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

    # ── [2, :]  Mapa de calor: detección × barrido ──────────
    ax = _ax(fig, gs[2, :])
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
            '(claro = ACQUIRED,  oscuro = NO LOCK)',
            fontsize=9, color=TITLE_CLR, pad=4)
        fig.colorbar(im, ax=ax, shrink=0.8)
    else:
        ax.text(0.5, 0.5, 'Esperando datos…',
                ha='center', va='center', color=TEXT_CLR, fontsize=10)
        ax.set_title('Mapa de detección', fontsize=9,
                     color=TITLE_CLR, pad=4)


# ──────────────────────────────────────────────────────────────
# VENTANA 3 — Espectro Doppler + Mapa de flags de adquisición
# ──────────────────────────────────────────────────────────────
def build_win3(num_prns):
    fig = plt.figure(figsize=(max(18, num_prns * 0.65), 9),
                     facecolor=BG, num='GPS Spectrum')
    fig.canvas.manager.set_window_title('GPS Adquisición — Espectro & Diagnóstico')
    return fig


def update_win3(frame, fig, state, num_prns):
    """
    Ventana 3: dos paneles.
      Arriba: Espectro Doppler — SNR vs frecuencia Doppler (kHz) para todos los PRNs.
              Picos altos = señal detectada. Visible a qué frecuencia Doppler están los sats.
      Abajo:  Mapa de flags (8 bits del campo 'f' por PRN).
              Celda verde = criterio pasa, roja = falla, gris = sin dato.
              Identifica qué bit está bloqueando cada PRN de un vistazo.
    """
    with state.lock:
        live     = dict(state.live)
        sc       = state.sweep_count
        tot      = state.total_sats

    fig.clear()
    fig.text(0.5, 0.978,
             'GPS ADQUISICIÓN  —  ESPECTRO DOPPLER & MAPA DE FLAGS',
             ha='center', color=TITLE_CLR, fontsize=13, fontweight='bold')

    gs = gridspec.GridSpec(2, 1, figure=fig,
                           left=0.06, right=0.98,
                           top=0.94, bottom=0.06,
                           hspace=0.50)

    prns = list(range(1, num_prns + 1))

    # ── Panel superior: Espectro Doppler ────────────────────
    ax_top = fig.add_subplot(gs[0])
    ax_top.set_facecolor(PANEL)
    for sp in ax_top.spines.values():
        sp.set_color(GRID_CLR)

    # Rango Doppler visible según CFG_BIN_RANGE (320 Hz/bin).
    bin_range = 124
    if HAS_FPGA_CFG:
        try:
            cfg = _parse_fpga_config()
            bin_range = int(cfg.get('CFG_BIN_RANGE', 124))
        except Exception:
            bin_range = 124
    DOPPLER_RANGE_KHZ = (bin_range * 320) / 1000.0
    ax_top.set_xlim(-DOPPLER_RANGE_KHZ, DOPPLER_RANGE_KHZ)
    ax_top.axvline(0, color=GRID_CLR, lw=1.2, ls='--', alpha=0.6,
                   label='IF=DC (después de decimación ×16)')
    # Banda orbital GPS ±4.5 kHz (sombreado verde ténue)
    ax_top.axvspan(-4.5, 4.5, color=GOOD_CLR, alpha=0.05,
                   label='Doppler orbital GPS ±4.5 kHz')
    # Banda de oscilador ±15.7 kHz (sombreado amarillo ténue)
    ax_top.axvspan(-15.7, 15.7, color=WARN_CLR, alpha=0.04,
                   label='Oscilador MAX2769C ±10 ppm (±15.7 kHz)')

    max_snr_seen = SNR_MIN_DISPLAY
    for prn, r in live.items():
        snr = r.get('snr', 0)
        if r.get('acquired') and snr > 0:
            max_snr_seen = max(max_snr_seen, snr)
    ax_top.set_ylim(0, max(SNR_MIN_DISPLAY, int(max_snr_seen * 1.3)))

    for prn, r in live.items():
        dop_hz  = r.get('doppler', 0)
        dop_khz = dop_hz / 1000.0
        snr     = r.get('snr', 0)
        acq     = r.get('acquired', False)
        if acq and snr > 0:
            clr = ACQ_CLR
            lw  = 2.0
            ax_top.vlines(dop_khz, 0, snr, colors=clr, linewidth=lw, zorder=3)
            ax_top.plot(dop_khz, snr, 'o', color=clr, ms=5, zorder=4)
            ax_top.text(dop_khz, snr * 1.04, f'P{prn:02d}',
                        ha='center', fontsize=6, color=clr, fontweight='bold')
        else:
            # PRN no adquirido: marca pequeña en la base si hay dato de Doppler
            flags = r.get('flags', 0)
            # Solo dibujar marca si hay datos válidos (flags > 0 indica respuesta recibida)
            if flags > 0:
                ax_top.plot(dop_khz, max_snr_seen * 0.015, 'v',
                            color=BAD_CLR, ms=4, alpha=0.5, zorder=2)

    ax_top.set_xlabel('Doppler (kHz)', fontsize=9)
    ax_top.set_ylabel('SNR proxy (adim.)', fontsize=9)
    ax_top.yaxis.grid(True)
    ax_top.set_axisbelow(True)
    ax_top.legend(fontsize=7, facecolor=PANEL, edgecolor=GRID_CLR,
                  labelcolor=TEXT_CLR, loc='upper right')
    ax_top.set_title(
        f'Espectro Doppler  —  Sweep #{sc}  '
        f'ACQUIRED: {tot}/{num_prns}',
        fontsize=9, color=TITLE_CLR, pad=4)

    # ── Panel inferior: Mapa de flags ───────────────────────
    ax_bot = fig.add_subplot(gs[1])
    ax_bot.set_facecolor(PANEL)
    for sp in ax_bot.spines.values():
        sp.set_color(GRID_CLR)

    n_bits = len(FLAG_NAMES)
    mat    = np.full((n_bits, num_prns), -1.0)  # -1 = sin dato

    for i, prn in enumerate(prns):
        r = live.get(prn)
        if r is not None:
            f = r.get('flags', -1)
            if f >= 0:
                for b in range(n_bits):
                    mat[b, i] = float((f >> b) & 1)

    # Construir imagen RGB manualmente: verde=pasa, azul=falla, gris=sin dato
    # Shape (n_bits, num_prns, 3)
    GREEN = np.array([0.094, 0.612, 0.243])  # GOOD_CLR #189C3E
    RED   = np.array([0.059, 0.376, 0.655])  # BAD_CLR  #0F60A7
    GRAY  = np.array([0.322, 0.337, 0.408])  # NOLOCK   #525668

    img = np.zeros((n_bits, num_prns, 3))
    for b in range(n_bits):
        for i in range(num_prns):
            v = mat[b, i]
            if v < 0:
                img[b, i] = GRAY
            elif v > 0.5:
                img[b, i] = GREEN
            else:
                img[b, i] = RED

    # Mostrar con bit7=lock arriba, bit0=abs_thr abajo
    ax_bot.imshow(img[::-1], aspect='auto', interpolation='nearest',
                  extent=[-0.5, num_prns - 0.5, -0.5, n_bits - 0.5])

    ax_bot.set_xticks(range(num_prns))
    ax_bot.set_xticklabels([f'{p:02d}' for p in prns], fontsize=6)
    ax_bot.set_yticks(range(n_bits))
    ax_bot.set_yticklabels(FLAG_NAMES[::-1], fontsize=7)  # bit7 arriba
    ax_bot.set_xlabel('PRN', fontsize=9)
    ax_bot.set_title(
        'Mapa de flags por PRN  '
        '(verde=pasa ✔  azul=falla ✘  gris=sin dato)',
        fontsize=9, color=TITLE_CLR, pad=4)

    # Leyenda compacta debajo
    legend_items = [
        mpatches.Patch(facecolor=GOOD_CLR, label='Bit=1  (criterio pasa)'),
        mpatches.Patch(facecolor=BAD_CLR, label='Bit=0  (criterio falla)'),
        mpatches.Patch(facecolor=NOLOCK, label='Sin dato'),
    ]
    ax_bot.legend(handles=legend_items, loc='lower right',
                  facecolor=PANEL, edgecolor=GRID_CLR,
                  labelcolor=TEXT_CLR, fontsize=7, framealpha=1, ncol=3)


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
    ap.add_argument('--interval3',       type=int, default=1000,
                    help='Refresco ventana espectro & flags (ms)')
    ap.add_argument('--no-spectrum',     action='store_true',
                    help='No abrir Ventana 3 (espectro & flags)')
    ap.add_argument('--demo',      action='store_true',
                    help='Datos simulados sin hardware')
    ap.add_argument('--log',       type=str, default=None,
                    help='Archivo para guardar el log UART en vivo')
    ap.add_argument('--replay',    type=str, default=None,
                    help='Archivo de log para reproducir offline')
    ap.add_argument('--speed',     type=float, default=1.0,
                    help='Velocidad de reproducción offline')
    args = ap.parse_args()

    # ── Carga de parametros FPGA ─────────────────────────────────────────
    if HAS_FPGA_CFG:
        try:
            _print_fpga_config()
        except Exception:
            pass

    state = State(args.prns)
    stop  = threading.Event()

    if args.replay:
        print(f'Modo REPLAY — reproduciendo {args.replay}')
        t = threading.Thread(target=file_reader,
                             args=(args.replay, state, stop, args.speed), daemon=True)
    elif args.demo:
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
                             args=(port, args.baud, state, stop, args.log), daemon=True)
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

    # Ventana 3 — Espectro Doppler + Mapa de flags
    ani3 = None
    if not args.no_spectrum:
        fig3 = build_win3(args.prns)
        ani3 = FuncAnimation(fig3, update_win3,
                             fargs=(fig3, state, args.prns),
                             interval=args.interval3, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        print('Monitor cerrado.')


if __name__ == '__main__':
    main()