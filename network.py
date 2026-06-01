"""
network.py — thread-safe shared memory, REST API polling, UDP tracking, and message sender.
"""

import json
import socket
import threading
import time
import urllib.request
import urllib.error
import os
from typing import Optional

from config import (
    COORD_SOURCE, COORD_FILE_PATH, COORD_POLL_INTERVAL, LISTEN_PORT,
    REST_API_GARBAGE, REST_API_BASKET, REST_API_TIMEOUT, REST_API_KEY,
    COMPUTER_IP, COMPUTER_PORT, FIXED_BASKET_X, FIXED_BASKET_Y
)

class SharedMemory:
    def __init__(self):
        self._lock = threading.Lock()
        self._target = None; self._disposal = None; self._robot_position = None
        self._target_lock_flag = 0

    def update_target(self, target: dict):
        with self._lock: self._target = target
    def get_target(self) -> Optional[dict]:
        with self._lock: return self._target
    def clear_target(self):
        with self._lock: self._target = None

    def set_target_lock_flag(self, value: int):
        with self._lock: self._target_lock_flag = 1 if value else 0
    def get_target_lock_flag(self) -> int:
        with self._lock: return self._target_lock_flag

    def update_disposal(self, disposal: dict):
        with self._lock: self._disposal = disposal
    def get_disposal(self) -> Optional[dict]:
        with self._lock: return self._disposal
    def clear_disposal(self):
        with self._lock: self._disposal = None

    def update_robot_position(self, pos: dict):
        with self._lock: self._robot_position = pos
    def get_and_clear_robot_position(self) -> Optional[dict]:
        with self._lock:
            pos = self._robot_position
            self._robot_position = None
            return pos

class MessageSender:
    """Handles sending status messages back to the computer."""
    @staticmethod
    def send_status(status: str, position: dict, message: str = ""):
        try:
            payload = {"status": status, "position": position, "message": message, "timestamp": time.time()}
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(json.dumps(payload).encode('utf-8'), (COMPUTER_IP, COMPUTER_PORT))
            sock.close()
            print(f"[Net] Sent to computer: {payload}")
        except Exception as e:
            print(f"[Net] Failed to send message: {e}")

class NetworkLayer(threading.Thread):
    def __init__(self, shared_memory: SharedMemory, state_callback):
        super().__init__(daemon=True)
        self.memory = shared_memory
        self.state_callback = state_callback
        
    def _udp_listener(self):
        """Listens for live position tracking from streaming_sender.py"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', LISTEN_PORT))
        print(f"[Net] UDP Listener started on port {LISTEN_PORT}")
        while True:
            try:
                data, _ = sock.recvfrom(2048)
                cmd = json.loads(data.decode('utf-8'))
                
                # Update live position from external tracking
                if cmd.get("type") == "position":
                    self.memory.update_robot_position(cmd)
            except Exception:
                pass

    def _polling_loop(self):
        """Polls coordinates depending on the FSM State and COORD_SOURCE"""
        last_garbage = None; last_basket = None
        
        while True:
            state = self.state_callback()
            
            if COORD_SOURCE == 'rest':
                # --- REST API MODE ---
                if state in ['IDLE', 'NAVIGATE_EXTERNAL']:
                    try:
                        req = urllib.request.Request(REST_API_GARBAGE)
                        if REST_API_KEY: req.add_header('Authorization', f'Bearer {REST_API_KEY}')
                        with urllib.request.urlopen(req, timeout=REST_API_TIMEOUT) as resp:
                            cmd = json.loads(resp.read().decode('utf-8'))
                        
                        # THE FIX: Check that x/y exist AND valid is exactly 1
                        if 'x' in cmd and 'y' in cmd and cmd.get('valid') == 1:
                            if cmd != last_garbage:
                                self.memory.update_target(cmd)
                                last_garbage = cmd
                        else:
                            # If valid is 0 or missing, erase the ghost target!
                            self.memory.clear_target()
                            
                    except Exception: 
                        self.memory.clear_target() # Clear on network fail
                
                if state in ['HOLDING', 'NAVIGATE_DISPOSAL', 'DISPOSE']:
                    try:
                        req = urllib.request.Request(REST_API_BASKET)
                        if REST_API_KEY: req.add_header('Authorization', f'Bearer {REST_API_KEY}')
                        with urllib.request.urlopen(req, timeout=REST_API_TIMEOUT) as resp:
                            cmd = json.loads(resp.read().decode('utf-8'))
                            
                        # THE FIX: Check that x/y exist AND valid is exactly 1
                        if 'x' in cmd and 'y' in cmd and cmd.get('valid') == 1:
                            if cmd != last_basket or self.memory.get_disposal() is None:
                                self.memory.update_disposal(cmd)
                                last_basket = cmd
                        else:
                            self.memory.clear_disposal()
                            
                    except Exception: 
                        pass # Don't clear disposal on timeout, just wait

            elif COORD_SOURCE == 'file':
                # --- LOCAL FILE MODE ---
                if state in ['IDLE', 'NAVIGATE_EXTERNAL']:
                    try:
                        if os.path.exists(COORD_FILE_PATH):
                            with open(COORD_FILE_PATH, 'r') as f:
                                cmd = json.load(f)
                                
                            # THE FIX: File check for valid=1
                            if 'x' in cmd and 'y' in cmd and cmd.get('valid') == 1:
                                if cmd != last_garbage:
                                    self.memory.update_target(cmd)
                                    last_garbage = cmd
                            else:
                                self.memory.clear_target()
                    except Exception as e: 
                        self.memory.clear_target()
                
                if state in ['HOLDING', 'NAVIGATE_DISPOSAL', 'DISPOSE']:
                    fixed_cmd = {'x': FIXED_BASKET_X, 'y': FIXED_BASKET_Y, 'z': 0.0}
                    if self.memory.get_disposal() is None:
                        self.memory.update_disposal(fixed_cmd)

            time.sleep(COORD_POLL_INTERVAL)

    def run(self):
        # Start coordinate polling in the background
        threading.Thread(target=self._polling_loop, daemon=True).start()
        # Run UDP live-tracking listener in this thread
        self._udp_listener()