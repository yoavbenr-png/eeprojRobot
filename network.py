"""
network.py — thread-safe shared memory, UDP listener, and message sender.
Supports UDP, file-based, and REST API coordinate reading.
"""

import json
import os
import socket
import threading
import time
import urllib.request
import urllib.error
from typing import Optional

from config import (
    LISTEN_PORT, COMPUTER_IP, COMPUTER_PORT,
    COORD_SOURCE, COORD_FILE_PATH, COORD_POLL_INTERVAL,
    REST_API_URL, REST_API_TIMEOUT, REST_API_KEY,
)


class SharedMemory:
    """Thread-safe storage for current navigation target and disposal location."""

    def __init__(self):
        self._lock   = threading.Lock()
        self._target: Optional[dict] = None
        self._disposal: Optional[dict] = None

    def update_target(self, target: dict):
        """Update the trash target coordinates."""
        with self._lock:
            self._target = target

    def get_target(self) -> Optional[dict]:
        """Get current trash target coordinates."""
        with self._lock:
            return self._target

    def clear_target(self):
        """Clear the trash target (after pickup)."""
        with self._lock:
            self._target = None

    def update_disposal(self, disposal: dict):
        """Update the disposal location coordinates."""
        with self._lock:
            self._disposal = disposal

    def get_disposal(self) -> Optional[dict]:
        """Get disposal location coordinates."""
        with self._lock:
            return self._disposal

    def clear_disposal(self):
        """Clear disposal location (after disposal complete)."""
        with self._lock:
            self._disposal = None


class MessageSender:
    """Handles sending status messages back to the computer."""

    @staticmethod
    def send_status(status: str, position: dict, message: str = ""):
        """
        Send a status message to the computer via UDP.
        
        Args:
            status: Status code (e.g., 'trash_collected', 'disposal_complete')
            position: Current robot position {'x': float, 'y': float}
            message: Human-readable message
        """
        try:
            payload = {
                "status": status,
                "position": position,
                "message": message,
                "timestamp": time.time()
            }
            
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(
                json.dumps(payload).encode('utf-8'),
                (COMPUTER_IP, COMPUTER_PORT)
            )
            sock.close()
            
            print(f"[Net] Sent to computer: {payload}")
            
        except Exception as e:
            print(f"[Net] Failed to send message: {e}")


class NetworkLayer(threading.Thread):
    """
    Daemon thread that receives target coordinates via UDP or file polling.
    
    Packet format:
        {"x": <float>, "y": <float>, "z": <float (optional)>, "type": "trash" or "disposal"}
    
    Type field:
        - "trash" (or omitted): trash target coordinates
        - "disposal": disposal location coordinates
    """

    def __init__(self, shared_memory: SharedMemory):
        super().__init__(daemon=True)
        self.memory = shared_memory
        
        if COORD_SOURCE == 'udp':
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.bind(('0.0.0.0', LISTEN_PORT))
            self._sock.settimeout(1.0)  # Allow periodic checks
        else:
            self._sock = None

    def _handle_packet(self, cmd: dict, source: str = "unknown"):
        """Process incoming coordinate packet."""
        if 'x' not in cmd or 'y' not in cmd:
            print(f"[Net] Ignored packet from {source} (missing x/y): {cmd}")
            return

        cmd.setdefault('z', 0.0)
        
        # Simple format: just coordinates
        # Robot determines meaning based on its current state
        # - If IDLE/NAVIGATING → trash target
        # - If HOLDING → disposal location
        
        self.memory.update_target(cmd)
        print(f"[Net] New coordinates from {source}: "
              f"({cmd['x']:.3f}, {cmd['y']:.3f}, {cmd['z']:.3f})")

    def _udp_mode(self):
        """Run UDP listener mode."""
        print(f"[Net] Listening on UDP port {LISTEN_PORT}")
        while True:
            try:
                data, addr = self._sock.recvfrom(4096)
                cmd = json.loads(data.decode('utf-8'))
                self._handle_packet(cmd, source=str(addr))

            except socket.timeout:
                continue  # Normal timeout, keep listening
            except json.JSONDecodeError as e:
                print(f"[Net] JSON decode error: {e}")
            except Exception as e:
                print(f"[Net] Error: {e}")

    def _file_mode(self):
        """Run file polling mode."""
        print(f"[Net] Polling coordinate file: {COORD_FILE_PATH}")
        last_mtime = 0

        while True:
            try:
                if os.path.exists(COORD_FILE_PATH):
                    mtime = os.path.getmtime(COORD_FILE_PATH)
                    
                    # Only read if file was modified
                    if mtime > last_mtime:
                        with open(COORD_FILE_PATH, 'r') as f:
                            cmd = json.load(f)
                        
                        self._handle_packet(cmd, source="file")
                        last_mtime = mtime

            except json.JSONDecodeError as e:
                print(f"[Net] File JSON error: {e}")
            except Exception as e:
                print(f"[Net] File read error: {e}")

            time.sleep(COORD_POLL_INTERVAL)

    def _rest_mode(self):
        """
        Poll the REST API every COORD_POLL_INTERVAL seconds.
        Expected JSON: {"x": 0.5, "y": 0.0, "z": 0.0}
        Only forwards to SharedMemory when coordinates actually change.
        On startup, does one silent poll to ignore stale coordinates
        from a previous run.
        """
        print(f"[Net] REST mode — polling {REST_API_URL} every {COORD_POLL_INTERVAL}s")

        # Startup: sample once silently so we don't act on stale coords
        last_coords = None
        print(f"[Net] Startup: sampling API to discard stale coordinates...")
        try:
            req = urllib.request.Request(REST_API_URL)
            if REST_API_KEY:
                req.add_header('Authorization', f'Bearer {REST_API_KEY}')
            with urllib.request.urlopen(req, timeout=REST_API_TIMEOUT) as resp:
                cmd = json.loads(resp.read().decode('utf-8'))
            if 'x' in cmd and 'y' in cmd:
                cmd.setdefault('z', 0.0)
                last_coords = (cmd['x'], cmd['y'], cmd['z'])
                print(f"[Net] Startup: ignoring existing {last_coords} — waiting for new coordinate")
        except Exception as e:
            print(f"[Net] Startup poll failed ({e}) — will act on first response received")

        while True:
            try:
                req = urllib.request.Request(REST_API_URL)
                if REST_API_KEY:
                    req.add_header('Authorization', f'Bearer {REST_API_KEY}')

                with urllib.request.urlopen(req, timeout=REST_API_TIMEOUT) as resp:
                    cmd = json.loads(resp.read().decode('utf-8'))

                if 'x' not in cmd or 'y' not in cmd:
                    print(f"[Net] REST response missing x/y: {cmd}")
                else:
                    cmd.setdefault('z', 0.0)
                    current = (cmd['x'], cmd['y'], cmd['z'])
                    if current != last_coords:
                        self._handle_packet(cmd, source="REST API")
                        last_coords = current
                    else:
                        print(f"[Net] REST: no change ({cmd['x']:.3f}, {cmd['y']:.3f}) — skipping")

            except urllib.error.URLError as e:
                print(f"[Net] REST request failed: {e.reason}")
            except json.JSONDecodeError as e:
                print(f"[Net] REST JSON error: {e}")
            except Exception as e:
                print(f"[Net] REST unexpected error: {e}")

            time.sleep(COORD_POLL_INTERVAL)

    def run(self):
        """Main thread loop — selects mode from COORD_SOURCE in config.py."""
        if COORD_SOURCE == 'udp':
            self._udp_mode()
        elif COORD_SOURCE == 'rest':
            self._rest_mode()
        else:
            self._file_mode()