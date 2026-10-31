#!/usr/bin/env python3
"""Live scrolling plot of the WAS test firmware output (mV on D4).

Usage: python3 plot.py [port|auto] [window_seconds]
Keys in the plot window: r = reset min/max on the ESP32, c = clear history.
"""
import glob
import re
import sys
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

PORT = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] != 'auto' else None  # None: first /dev/ttyUSB*
WINDOW_S = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
LINE_RE = re.compile(r'mV=\s*(\d+)\s+raw=\s*(\d+)')

ser = None
t0 = time.time()
ts, mvs = deque(), deque()
lock = threading.Lock()


def reader():
    # Reopen the port whenever it drops (replug, brownout, another program).
    global ser
    while True:
        try:
            if ser is None:
                ports = [PORT] if PORT else sorted(glob.glob('/dev/ttyUSB*'))
                if not ports:
                    raise serial.SerialException('no /dev/ttyUSB* found')
                ser = serial.Serial(ports[0], 115200, timeout=1)
                print(f'serial: opened {ports[0]}', flush=True)
                ser.reset_input_buffer()
            m = LINE_RE.search(ser.readline().decode(errors='ignore'))
        except (serial.SerialException, OSError) as e:
            print(f'serial: {e}; reconnecting...', flush=True)
            if ser is not None:
                ser.close()
            ser = None
            time.sleep(1)
            continue
        if not m:
            continue
        with lock:
            ts.append(time.time() - t0)
            mvs.append(int(m.group(1)))


threading.Thread(target=reader, daemon=True).start()

fig, ax = plt.subplots(figsize=(10, 5))
line, = ax.plot([], [], lw=1.5)
ax.axhline(3100, color='r', ls='--', lw=0.8, label='ADC limit (~3.1 V)')
ax.set_ylim(0, 3300)
ax.set_xlabel('time [s]')
ax.set_ylabel('D4 [mV]')
ax.grid(True, alpha=0.3)
ax.legend(loc='upper left')
info = ax.text(0.99, 0.97, '', transform=ax.transAxes, ha='right', va='top',
               family='monospace', bbox=dict(fc='white', alpha=0.8))


def on_key(event):
    if event.key == 'r' and ser is not None:
        ser.write(b'r')
    elif event.key == 'c':
        with lock:
            ts.clear()
            mvs.clear()


fig.canvas.mpl_connect('key_press_event', on_key)


def update(_):
    with lock:
        if not ts:
            return line, info
        x, y = list(ts), list(mvs)
    now = x[-1]
    ax.set_xlim(max(0, now - WINDOW_S), max(WINDOW_S, now))
    line.set_data(x, y)
    vis = [v for t, v in zip(x, y) if t >= now - WINDOW_S]
    info.set_text(f'now {y[-1]:4d} mV\nmin {min(vis):4d}\nmax {max(vis):4d}\n'
                  f'span {max(vis) - min(vis):4d}')
    return line, info


anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
plt.title(f'WAS live — {PORT or "auto"}')
plt.show()
