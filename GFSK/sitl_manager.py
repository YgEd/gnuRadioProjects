"""
SITL Manager
=============
Starts an ArduCopter SITL instance, connects to it via MAVLink,
forwards decoded packets from the RF chain, and logs all telemetry
(heartbeats, STATUSTEXT, GPS, etc.) to a separate CSV alongside
the RF metrics log.

Architecture
-------------
                    ┌─────────────────────────────┐
                    │       sitl_manager.py        │
                    │                              │
  mav_packet_reader_with_metrics                   │
        │ (success/fail)                           │
        ▼                                          │
  MetricsLogger  ──── update_iq_metrics ──────────►│  RFMetricsProbe
        │                                          │
        │  log_packet_outcome()                    │
        ▼                                          │
  SITLManager.forward_packet(bytes)  ◄─────────────┘
        │
        │  (via mp.Queue to SITL process)
        ▼
  _sitl_process()  ←─ subprocess.Popen(sim_vehicle.py)
        │
        ├── sends heartbeat every 1s
        ├── forwards packets from forward_queue
        ├── logs HEARTBEAT / STATUSTEXT / GPS to CSV
        └── puts telemetry dicts into telemetry_queue
              │
              ▼
        SITLManager.get_latest_telemetry()
              │
              ▼
        Your main script / MetricsLogger

Usage in rx.py
---------------
    from sitl_manager import SITLManager

    sitl = SITLManager(
        sitl_address='udp:127.0.0.1:14550',
        log_dir='packet-logs',
        auto_start=True           # launches sim_vehicle.py automatically
    )
    sitl.start()

    # In mav_packet_reader_with_metrics, replace self.master.write() with:
    sitl.forward_packet(payload_bytes)

    # On shutdown:
    sitl.stop()
"""

import multiprocessing as mp
import subprocess
import threading
import time
import os
import csv
import datetime
import signal
import sys

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


# ---------------------------------------------------------------------------
# SITL PROCESS — runs in its own process, fully isolated from GR/Qt threads
# ---------------------------------------------------------------------------

def _sitl_process(
    sitl_address: str,
    forward_queue: mp.Queue,
    hb_queue,
    telemetry_queue: mp.Queue,
    stop_event: mp.Event,
    log_dir: str,
):
    """
    Target function for the SITL worker process.

    Responsibilities:
      1. Connect to the SITL MAVLink endpoint
      3. Drain forward_queue and write raw MAVLink bytes to SITL
      4. Receive all MAVLink messages from SITL, log them, and push
         important ones into telemetry_queue for the main process
    """

    print(f"[SITL] Connecting to {sitl_address} ...")

    # Retry connection — SITL may take a few seconds to boot
    conn = None
    for attempt in range(10):
        try:
            conn = mavutil.mavlink_connection(sitl_address)
            hb = conn.wait_heartbeat(timeout=5)
            if hb:
                print(f"[SITL] Connected — vehicle sysid={conn.target_system} "
                      f"compid={conn.target_component}")
                break
        except Exception as e:
            print(f"[SITL] Attempt {attempt+1}/10 failed: {e}")
            time.sleep(2)
    else:
        print("[SITL] Could not connect after 10 attempts. Exiting process.")
        return

    # ── Configure SITL params once connected ──
    # Tell ArduCopter this system (sysid=255) is the primary GCS
    # param set requires a short delay after connection
    time.sleep(1)
    try:
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            b'SYSID_MYGCS',
            255.0,
            mavlink2.MAV_PARAM_TYPE_INT32
        )
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            b'FS_GCS_ENABLE',
            1.0,
            mavlink2.MAV_PARAM_TYPE_INT32
        )
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            b'FS_GCS_TIMEOUT',
            5.0,
            mavlink2.MAV_PARAM_TYPE_INT32
        )
        print("[SITL] Parameters configured")
    except Exception as e:
        print(f"[SITL] Param set failed (non-fatal): {e}")

    # ── Open telemetry log CSV ──
    os.makedirs(log_dir, exist_ok=True)
    now = datetime.datetime.now().isoformat()
    telem_path = os.path.join(log_dir, f'sitl-telemetry-{now}.csv')

    with open(telem_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp', 'msg_type',
            'armed', 'mode',
            'lat', 'lon', 'alt_m',
            'vx', 'vy', 'vz',
            'roll', 'pitch', 'yaw',
            'statustext',
            'link_quality',
        ])

    print(f"[SITL] Telemetry log: {telem_path}")

    last_telem = {
        'armed': False, 'mode': 0,
        'lat': None, 'lon': None, 'alt_m': None,
        'vx': None, 'vy': None, 'vz': None,
        'roll': None, 'pitch': None, 'yaw': None,
        'link_quality': None,
    }

    while not stop_event.is_set():

       

        # # ── 2. Forward decoded RF packets to SITL ──
        # while not forward_queue.empty():
        #     try:
        #         payload_bytes = forward_queue.get_nowait()
        #         conn.write(payload_bytes)
        #         print(f"[SITL] Forwarded {len(payload_bytes)}-byte packet to SITL")
        #     except Exception as e:
        #         print(f"[SITL] Forward error: {e}")

        # ── 3. Receive and process incoming MAVLink messages ──
        msg = conn.recv_match(blocking=False)
        if msg is None:
            time.sleep(0.005)
            continue

        t = msg.get_type()
        ts = datetime.datetime.now().isoformat()
        statustext = ''

        if t == 'HEARTBEAT':
            armed = bool(msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED)
            last_telem['armed'] = armed
            last_telem['mode'] = msg.custom_mode
            print(f"[SITL] HB — armed={armed} mode={msg.custom_mode} "
                  f"state={msg.system_status}\nPassing to mavGNUTXBlock to send")
            
            try:
                hb_queue.put_nowait({
                    'type': msg.type,
                    'autopilot': msg.autopilot,
                    'base_mode': msg.base_mode,
                    'custom_mode': msg.custom_mode,
                    'system_status': msg.system_status,
                    'mavlink_version': msg.mavlink_version,
                })
            except Exception as e:
                print(f"[SITL] When trying to push hearbeat into queue got error:\n{e}")
                pass
                
            

        elif t == 'STATUSTEXT':
            statustext = msg.text
            severity_map = {
                0: 'EMERGENCY', 1: 'ALERT', 2: 'CRITICAL', 3: 'ERROR',
                4: 'WARNING', 5: 'NOTICE', 6: 'INFO', 7: 'DEBUG'
            }
            sev = severity_map.get(msg.severity, str(msg.severity))
            print(f"[SITL] STATUS [{sev}]: {msg.text}")

            # Push link-related events to telemetry queue immediately
            text_lower = msg.text.lower()
            if any(kw in text_lower for kw in
                   ['link', 'failsafe', 'gcs', 'timeout', 'lost', 'rc']):
                telemetry_queue.put({
                    'type': 'link_event',
                    'severity': sev,
                    'text': msg.text,
                    'timestamp': ts,
                })

        # elif t == 'GLOBAL_POSITION_INT':
        #     last_telem['lat'] = msg.lat / 1e7
        #     last_telem['lon'] = msg.lon / 1e7
        #     last_telem['alt_m'] = msg.relative_alt / 1000.0
        #     last_telem['vx'] = msg.vx / 100.0
        #     last_telem['vy'] = msg.vy / 100.0
        #     last_telem['vz'] = msg.vz / 100.0

        # elif t == 'ATTITUDE':
        #     import math
        #     last_telem['roll']  = math.degrees(msg.roll)
        #     last_telem['pitch'] = math.degrees(msg.pitch)
        #     last_telem['yaw']   = math.degrees(msg.yaw)

        # elif t == 'RC_CHANNELS':
        #     # RSSI from RC link as crude link quality proxy (0–255)
        #     last_telem['link_quality'] = msg.rssi

        # ── 4. Write CSV row for all messages we care about ──
        # loggable = {'HEARTBEAT', 'STATUSTEXT', 'GLOBAL_POSITION_INT',
        #             'ATTITUDE', 'RC_CHANNELS'}
        
        loggable = {'HEARTBEAT', 'STATUSTEXT'}
        if t in loggable:
            with open(telem_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    ts, t,
                    last_telem['armed'],
                    last_telem['mode'],
                    last_telem['lat'],
                    last_telem['lon'],
                    last_telem['alt_m'],
                    last_telem['vx'],
                    last_telem['vy'],
                    last_telem['vz'],
                    last_telem['roll'],
                    last_telem['pitch'],
                    last_telem['yaw'],
                    statustext,
                    last_telem['link_quality'],
                ])

        # Push latest snapshot to main process at low rate
        try:
            telemetry_queue.put_nowait({'type': 'telem_snapshot', **last_telem})
        except Exception:
            pass 

    print("[SITL] Worker process exiting cleanly")


# ---------------------------------------------------------------------------
# SITL MANAGER — public API used by rx.py
# ---------------------------------------------------------------------------

class SITLManager:
    """
    Manages a SITL ArduCopter instance from within a GNU Radio pipeline.

    Parameters
    ----------
    sitl_address : str
        MAVLink UDP address, default 'udp:127.0.0.1:14550'
    log_dir : str
        Directory for telemetry CSV logs
    auto_start : bool
        If True, launches sim_vehicle.py as a subprocess before connecting.
        If False, assumes SITL is already running externally.
    sitl_vehicle : str
        Vehicle type passed to sim_vehicle.py, default 'ArduCopter'
    sitl_extra_args : list[str]
        Additional arguments forwarded to sim_vehicle.py
    """

    def __init__(
        self,
        sitl_address: str = 'udp:127.0.0.1:14550',
        log_dir: str = 'packet-logs',
        auto_start: bool = True,
        sitl_vehicle: str = 'ArduCopter',
        sitl_extra_args: list = None,
    ):
        self.sitl_address    = sitl_address
        self.log_dir         = log_dir
        self.auto_start      = auto_start
        self.sitl_vehicle    = sitl_vehicle
        self.sitl_extra_args = sitl_extra_args or []

        # IPC primitives
        self._forward_queue  = mp.Queue(maxsize=200)
        self._telemetry_queue = mp.Queue(maxsize=500)
        self._stop_event     = mp.Event()
        self._hb_queue = mp.Queue(maxsize=50)

        # SITL subprocess handle (sim_vehicle.py)
        self._sitl_proc: subprocess.Popen = None

        # MAVLink worker process
        self._mav_proc: mp.Process = None

        # Latest telemetry snapshot — updated by a drain thread in main process
        self._latest_telem = {}
        self._telem_lock   = threading.Lock()
        self._drain_thread: threading.Thread = None

    # ── Lifecycle ──────────────────────────────────────────────────────────

    def start(self):
        """Start SITL subprocess (optional) and MAVLink worker process."""

        if self.auto_start:
            self._launch_sitl()

        self._mav_proc = mp.Process(
            target=_sitl_process,
            args=(
                self.sitl_address,
                self._forward_queue,
                self._telemetry_queue,
                self._hb_queue,
                self._stop_event,
                self.log_dir,
            ),
            daemon=True,
            name='sitl-mavlink-worker'
        )
        self._mav_proc.start()
        print(f"[SITLManager] MAVLink worker PID={self._mav_proc.pid}")

        # Drain telemetry queue into _latest_telem in a lightweight thread
        self._drain_thread = threading.Thread(
            target=self._drain_telemetry,
            daemon=True,
            name='sitl-telem-drain'
        )
        self._drain_thread.start()

    def stop(self):
        """Gracefully shut down worker process and optionally kill SITL."""
        print("[SITLManager] Stopping...")
        self._stop_event.set()

        if self._mav_proc and self._mav_proc.is_alive():
            self._mav_proc.join(timeout=5)
            if self._mav_proc.is_alive():
                self._mav_proc.kill()

        if self._sitl_proc and self._sitl_proc.poll() is None:
            print("[SITLManager] Terminating sim_vehicle.py ...")
            self._sitl_proc.terminate()
            try:
                self._sitl_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._sitl_proc.kill()

        print("[SITLManager] Stopped.")

    # ── Public API ─────────────────────────────────────────────────────────

    def get_heartbeat(self):
        try:
            return self._hb_queue.get_nowait()
        except Exception as e:
            print(f"[SITLManager] Tried to get hearbeat but got error:\n{e}")
            return None
    
    def forward_packet(self, payload_bytes: bytes):
        """
        Called by mav_packet_reader_with_metrics on each successfully
        decoded RF packet. Queues bytes for forwarding to SITL.
        Non-blocking — drops packet if queue is full (avoids blocking GR work()).
        """
        try:
            self._forward_queue.put_nowait(payload_bytes)
        except Exception:
            print("[SITLManager] Forward queue full — packet dropped")

    def get_latest_telemetry(self) -> dict:
        """Returns a snapshot of the most recent telemetry from SITL."""
        with self._telem_lock:
            return dict(self._latest_telem)

    def is_armed(self) -> bool:
        return self.get_latest_telemetry().get('armed', False)

    def is_connected(self) -> bool:
        return self._mav_proc is not None and self._mav_proc.is_alive()

    # ── Internal ───────────────────────────────────────────────────────────

    def _launch_sitl(self):
        cmd = [
            'python3',
            os.path.expanduser('~/Desktop/ardupilot/Tools/autotest/sim_vehicle.py'),
            '-v', self.sitl_vehicle,
            '--no-rebuild',
            '--out=127.0.0.1:14550',
            '--out=127.0.0.1:14551',
            '--param=SYSID_MYGCS=255',
            '--param=FS_GCS_ENABLE=1',
            '--param=FS_GCS_TIMEOUT=5',
            '--mavproxy-args=--cmd="set heartbeat 0; set srcSystem 255"',
        ] + self.sitl_extra_args

        print(f"[SITLManager] Launching: {' '.join(cmd)}")
        self._sitl_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

        def _log_sitl_output(proc):
            for line in iter(proc.stdout.readline, b''):
                print(f"[SITL-SIM] {line.decode(errors='replace').rstrip()}")

        threading.Thread(
            target=_log_sitl_output,
            args=(self._sitl_proc,),
            daemon=True,
            name='sitl-stdout-reader'
        ).start()

        print(f"[SITLManager] sim_vehicle.py PID={self._sitl_proc.pid}, "
            f"waiting 8s for boot ...")
        time.sleep(8)

    def _drain_telemetry(self):
        """
        Lightweight thread that drains the telemetry queue and keeps
        _latest_telem current. Runs in the main process so the Qt/GR
        threads can call get_latest_telemetry() safely.
        """
        while not self._stop_event.is_set():
            try:
                item = self._telemetry_queue.get(timeout=0.1)
                if item.get('type') == 'telem_snapshot':
                    with self._telem_lock:
                        self._latest_telem.update(item)
                elif item.get('type') == 'link_event':
                    # Log link events to stdout — hook here for MetricsLogger if desired
                    print(f"[SITLManager] LINK EVENT: [{item['severity']}] {item['text']}")
            except Exception:
                pass   # timeout or empty — normal