#!/usr/bin/env python3
"""Laptop GUI for the solbot4 (pd600) sliding toolbar.

The robot's ROS graph is loopback-only (domain 0), so this does not use DDS: it
copies toolbar_gui_remote.py to pd600 and runs it over ssh, exchanging JSON lines.

    python3 toolbar_gui.py            (or: bash pd600.sh gui)

Slider: drag, the target is sent on release (set_position, manual mode).
Keys: Left/Right nudge target -/+5 mm (Shift: 1 mm), c centre, space = HOLD.
Closing the window sends hold if the toolbar is enabled.
"""

import json
import os
import queue
import subprocess
import threading
import tkinter as tk
from tkinter import ttk

HOST = os.environ.get('PD600_HOST', 'pd600')
HERE = os.path.dirname(os.path.abspath(__file__))
REMOTE_PY = '~/ros2_ws5/toolbar_gui_remote.py'
REMOTE_RUN = ('source ~/ros2_ws4/pd600_env.sh && source ~/ros2_ws5/install/setup.bash && '
              f'exec python3 -u {REMOTE_PY}')

STATE_COLORS = {
    'at_target': '#2e7d32', 'moving': '#1565c0', 'holding': '#6a1b9a',
    'disabled': '#757575', 'no_sensor': '#ef6c00', 'estop': '#c62828', 'fault': '#c62828',
}


class Link:
    """ssh pipe to toolbar_gui_remote.py on pd600."""

    def __init__(self, inbox):
        self.inbox = inbox
        self.proc = None

    def start(self):
        self.inbox.put({'type': 'log', 'message': f'copying bridge to {HOST} ...'})
        with open(os.path.join(HERE, 'toolbar_gui_remote.py'), 'rb') as f:
            r = subprocess.run(['ssh', '-o', 'ConnectTimeout=5', HOST,
                                f'cat > {REMOTE_PY}'], stdin=f, capture_output=True)
        if r.returncode != 0:
            self.inbox.put({'type': 'link', 'up': False,
                            'message': f'ssh {HOST} failed: {r.stderr.decode().strip()}'})
            return
        self.proc = subprocess.Popen(
            ['ssh', '-T', '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=2',
             '-o', 'ServerAliveCountMax=3', HOST, f"bash -c '{REMOTE_RUN}'"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1)
        threading.Thread(target=self._read_out, daemon=True).start()
        threading.Thread(target=self._read_err, daemon=True).start()
        self.inbox.put({'type': 'link', 'up': True, 'message': f'connected to {HOST}'})

    def _read_out(self):
        for line in self.proc.stdout:
            try:
                self.inbox.put(json.loads(line))
            except ValueError:
                self.inbox.put({'type': 'log', 'message': line.rstrip()})
        self.inbox.put({'type': 'link', 'up': False, 'message': 'link closed'})

    def _read_err(self):
        for line in self.proc.stderr:
            if 'deprecated element' not in line:
                self.inbox.put({'type': 'log', 'message': 'pd600: ' + line.rstrip()})

    def send(self, command, position=0.0):
        if not self.alive():
            return False
        try:
            self.proc.stdin.write(json.dumps({'command': command, 'position': position}) + '\n')
            self.proc.stdin.flush()
            return True
        except (BrokenPipeError, OSError):
            return False

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def close(self):
        if self.alive():
            try:
                self.proc.stdin.close()
                self.proc.wait(timeout=3)
            except Exception:  # noqa: BLE001
                self.proc.kill()


class App:
    def __init__(self, root):
        self.root = root
        self.inbox = queue.Queue()
        self.link = Link(self.inbox)
        self.st = {}
        self.min_m, self.max_m = -0.10, 0.10
        self.dragging = False

        root.title(f'solbot4 toolbar ({HOST})')
        root.minsize(620, 420)
        pad = {'padx': 8, 'pady': 4}

        top = ttk.Frame(root)
        top.pack(fill='x', **pad)
        self.state_lbl = tk.Label(top, text='connecting', font=('TkDefaultFont', 18, 'bold'),
                                  fg='white', bg='#757575', width=12)
        self.state_lbl.pack(side='left')
        self.info_lbl = ttk.Label(top, text='', justify='left', font=('TkFixedFont', 10))
        self.info_lbl.pack(side='left', padx=12)

        # position bar: measured (red line) and target (blue triangle)
        self.canvas = tk.Canvas(root, height=60, bg='#fafafa', highlightthickness=0)
        self.canvas.pack(fill='x', padx=20, pady=(8, 0))
        self.canvas.bind('<Configure>', lambda e: self.draw())

        self.target_var = tk.DoubleVar(value=0.0)
        self.scale = tk.Scale(root, from_=-100, to=100, orient='horizontal', resolution=1,
                              variable=self.target_var, label='target [mm]  (- left / + right)',
                              length=560, showvalue=True, command=lambda v: self.draw())
        self.scale.pack(fill='x', padx=12)
        self.scale.bind('<ButtonPress-1>', lambda e: setattr(self, 'dragging', True))
        self.scale.bind('<ButtonRelease-1>', self.on_release)

        btns = ttk.Frame(root)
        btns.pack(fill='x', **pad)
        self.enable_btn = ttk.Button(btns, text='Enable', command=lambda: self.cmd('enable'))
        self.enable_btn.pack(side='left')
        ttk.Button(btns, text='Disable', command=lambda: self.cmd('disable')).pack(side='left')
        ttk.Button(btns, text='Centre', command=lambda: self.cmd('center')).pack(side='left', padx=(12, 0))
        ttk.Button(btns, text='Go to slider', command=self.send_target).pack(side='left')
        ttk.Button(btns, text='Auto', command=lambda: self.cmd('auto')).pack(side='left', padx=(12, 0))
        ttk.Button(btns, text='Reset fault', command=lambda: self.cmd('reset_fault')).pack(side='left')
        self.reconnect_btn = ttk.Button(btns, text='Reconnect', command=self.connect)
        self.reconnect_btn.pack(side='right')

        stop = tk.Button(root, text='HOLD  (space)', bg='#c62828', fg='white',
                         activebackground='#e53935', font=('TkDefaultFont', 14, 'bold'),
                         command=lambda: self.cmd('hold'))
        stop.pack(fill='x', padx=8, pady=4)

        self.log = tk.Text(root, height=8, state='disabled', font=('TkFixedFont', 9))
        self.log.pack(fill='both', expand=True, **pad)

        root.bind('<Left>', lambda e: self.nudge(-5))
        root.bind('<Right>', lambda e: self.nudge(5))
        root.bind('<Shift-Left>', lambda e: self.nudge(-1))
        root.bind('<Shift-Right>', lambda e: self.nudge(1))
        root.bind('<space>', lambda e: self.cmd('hold'))
        root.bind('c', lambda e: self.cmd('center'))
        root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.connect()
        root.after(50, self.poll)

    # --- commands -------------------------------------------------------------
    def connect(self):
        self.link.close()
        threading.Thread(target=self.link.start, daemon=True).start()

    def cmd(self, command, position=0.0):
        if not self.link.send(command, position):
            self.add_log(f'not connected, {command} not sent')
            return
        self.add_log(f'> {command}' + (f' {position * 1000:+.0f} mm' if command == 'set_position' else ''))

    def send_target(self):
        self.cmd('set_position', round(self.target_var.get()) / 1000.0)

    def on_release(self, _e):
        self.dragging = False
        self.send_target()

    def nudge(self, mm):
        v = max(self.min_m * 1000, min(self.max_m * 1000, self.target_var.get() + mm))
        self.target_var.set(v)
        self.send_target()

    def on_close(self):
        if self.st.get('enabled') and self.link.alive():
            self.link.send('hold')
        self.link.close()
        self.root.destroy()

    # --- incoming -------------------------------------------------------------
    def poll(self):
        try:
            while True:
                m = self.inbox.get_nowait()
                t = m.get('type')
                if t == 'state':
                    self.on_state(m)
                elif t == 'reply':
                    ok = 'ok' if m.get('success') else 'REJECTED'
                    self.add_log(f'< {m.get("command")}: {ok} {m.get("message", "")}')
                elif t == 'link':
                    self.add_log(m['message'])
                    if not m['up']:
                        self.state_lbl.config(text='no link', bg='#c62828')
                else:
                    self.add_log(m.get('message', str(m)))
        except queue.Empty:
            pass
        self.root.after(50, self.poll)

    def on_state(self, s):
        first = not self.st
        self.st = s
        if s.get('min_position') is not None and s.get('max_position') is not None:
            if (s['min_position'], s['max_position']) != (self.min_m, self.max_m):
                self.min_m, self.max_m = s['min_position'], s['max_position']
                self.scale.config(from_=round(self.min_m * 1000), to=round(self.max_m * 1000))
        if first:  # start the slider where the toolbar is, not at 0
            tgt = s.get('target_position')
            ref = tgt if tgt is not None else s.get('current_position')
            if ref is not None:
                self.target_var.set(round(ref * 1000))
        state = s.get('state', '?')
        self.state_lbl.config(text=state, bg=STATE_COLORS.get(state, '#757575'))

        def mm(v):
            return '   ---' if v is None else f'{v * 1000:+6.1f}'
        duty = s.get('duty')
        flags = ' '.join(k for k in ('enabled', 'sensor_ok', 'mqtt_connected', 'estop',
                                      'in_motion', 'fault_latched') if s.get(k))
        self.info_lbl.config(text=(
            f'pos {mm(s.get("current_position"))} mm   target {mm(s.get("target_position"))} mm   '
            f'fw goal {mm(s.get("firmware_goal"))} mm\n'
            f'raw {s.get("raw_position", 0):6.0f}   duty {0 if duty is None else duty:+.2f}   '
            f'mode {s.get("mode")}   fw_fault {s.get("firmware_fault")}\n'
            f'{flags}' + (f'\nFAULT: {s.get("fault_reason")}' if s.get('fault_reason') else '')))
        self.enable_btn.state(['disabled'] if s.get('enabled') else ['!disabled'])
        self.draw()

    # --- drawing --------------------------------------------------------------
    def x_of(self, m, w):
        return 10 + (m - self.min_m) / (self.max_m - self.min_m) * (w - 20)

    def draw(self):
        c = self.canvas
        c.delete('all')
        w = c.winfo_width()
        if w < 50:
            return
        c.create_rectangle(10, 22, w - 10, 38, fill='#e0e0e0', outline='#9e9e9e')
        for mm in range(round(self.min_m * 1000), round(self.max_m * 1000) + 1, 25):
            x = self.x_of(mm / 1000, w)
            c.create_line(x, 38, x, 44, fill='#9e9e9e')
            c.create_text(x, 52, text=str(mm), font=('TkDefaultFont', 7), fill='#616161')
        x = self.x_of(self.target_var.get() / 1000, w)
        c.create_polygon(x - 6, 6, x + 6, 6, x, 20, fill='#90caf9', outline='#1565c0')
        tgt = self.st.get('target_position')
        if tgt is not None:
            x = self.x_of(tgt, w)
            c.create_polygon(x - 6, 6, x + 6, 6, x, 20, fill='#1565c0')
        pos = self.st.get('current_position')
        if pos is not None:
            x = self.x_of(pos, w)
            c.create_rectangle(x - 3, 16, x + 3, 44, fill='#c62828', outline='')

    def add_log(self, text):
        self.log.config(state='normal')
        self.log.insert('end', text + '\n')
        self.log.see('end')
        self.log.config(state='disabled')


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
