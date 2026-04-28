#!/usr/bin/env python3
"""
GCS Serial Bridge
Reads the ESP32 LoRa receiver's serial output (text lines) and
broadcasts each complete telemetry frame as JSON over WebSocket.

Install:   pip install pyserial websockets
Usage:     python gcs_bridge.py --port /dev/ttyUSB0
           python gcs_bridge.py --port COM5 --baud 115200 --ws-port 8765
           python gcs_bridge.py --list-ports
"""

import argparse
import asyncio
import json
import re
import time
from typing import Optional
import serial
import serial.tools.list_ports
import websockets
from websockets.server import serve

# ── Connected WebSocket clients ──────────────────────────────────────────────
CLIENTS: set = set()
MAX_COMMAND_BYTES = 240
SERIAL_CONN: Optional[serial.Serial] = None
SERIAL_WRITE_LOCK = asyncio.Lock()

# ── Regex patterns ────────────────────────────────────────────────────────────
# Matches the existing PrintPacket() output PLUS the new lines from receiver_patch.cpp
PATTERNS = {
    # [N] RSSI=-85 dBm | SNR=7.50 | mode=3 | sats=9 | lock=1
    'header': re.compile(
        r'\[(\d+)\] RSSI=(-?\d+) dBm \| SNR=(-?[\d.]+) \| mode=([\d.]+) \| sats=(-?\d+) \| lock=(-?\d+)'),
    # Attitude  roll=1.20  pitch=-0.50  yaw=45.00
    'attitude': re.compile(
        r'Attitude\s+roll=(-?[\d.]+)\s+pitch=(-?[\d.]+)\s+yaw=(-?[\d.]+)'),
    # Setpoint  roll=0.00  pitch=0.00  yaw=45.00   (NEW — from patch)
    'setpoint': re.compile(
        r'Setpoint\s+roll=(-?[\d.]+)\s+pitch=(-?[\d.]+)\s+yaw=(-?[\d.]+)'),
    # Control   thr=0.65  des_thr=0.70  airspeed=18.50  (NEW — from patch)
    'control': re.compile(
        r'Control\s+thr=(-?[\d.]+)\s+des_thr=(-?[\d.]+)\s+airspeed=(-?[\d.]+)'),
    # Altitude  alt=100.50  baro=100.20  target=100.00
    'altitude': re.compile(
        r'Altitude\s+alt=(-?[\d.]+)\s+baro=(-?[\d.]+)\s+target=(-?[\d.]+)'),
    # GPS       lat=40.427000  lon=-86.923000  alt=95.00  spd=15.20  hdg=45.00
    'gps': re.compile(
        r'GPS\s+lat=(-?[\d.]+)\s+lon=(-?[\d.]+)\s+alt=(-?[\d.]+)\s+spd=(-?[\d.]+)\s+hdg=(-?[\d.]+)'),
    # Waypoint  dist=150.00  hdg=90.00  idx=2/5  done=0
    'wp': re.compile(
        r'Waypoint\s+dist=(-?[\d.]+)\s+hdg=(-?[\d.]+)\s+idx=(\d+)/(\d+)\s+done=(-?\d+)'),
    # WPTarget  lat=40.430000  lon=-86.920000  alt=100.00  leg=0.45  miss=0.30  (NEW — from patch)
    'wptarget': re.compile(
        r'WPTarget\s+lat=(-?[\d.]+)\s+lon=(-?[\d.]+)\s+alt=(-?[\d.]+)\s+leg=(-?[\d.]+)\s+miss=(-?[\d.]+)'),
    # Failsafe  fs=0  (NEW — from patch)
    'failsafe': re.compile(r'Failsafe\s+fs=(-?[\d.]+)'),
    # ActivePID roll_p=...
    'active_pid': re.compile(
        r'ActivePID\s+roll_p=(-?[\d.]+)\s+roll_i=(-?[\d.]+)\s+roll_d=(-?[\d.]+)\s+pitch_p=(-?[\d.]+)\s+pitch_i=(-?[\d.]+)\s+pitch_d=(-?[\d.]+)\s+yaw_p=(-?[\d.]+)\s+yaw_i=(-?[\d.]+)\s+yaw_d=(-?[\d.]+)'),
    # NavPID alt_p=...
    'nav_pid': re.compile(
        r'NavPID\s+alt_p=(-?[\d.]+)\s+alt_i=(-?[\d.]+)\s+alt_d=(-?[\d.]+)\s+hdg_p=(-?[\d.]+)\s+hdg_i=(-?[\d.]+)\s+hdg_d=(-?[\d.]+)'),
}


def parse_line(line: str, packet: dict) -> bool:
    """
    Match *line* against known patterns and update *packet* in-place.
    Returns True only for header lines (signals start of a new packet group).
    """
    line = line.strip()
    if not line:
        return False

    m = PATTERNS['header'].search(line)
    if m:
        packet.update({
            'packet_id':         int(m.group(1)),
            'rssi':              int(m.group(2)),
            'snr':               float(m.group(3)),
            'flightmode':        float(m.group(4)),
            'gps_sats':          int(m.group(5)),
            'gps_lock_acquired': int(m.group(6)),
            'ts':                time.time(),
        })
        return True

    m = PATTERNS['attitude'].search(line)
    if m:
        packet.update({'roll': float(m.group(1)), 'pitch': float(m.group(2)), 'yaw': float(m.group(3))})
        return False

    m = PATTERNS['setpoint'].search(line)
    if m:
        packet.update({'des_roll': float(m.group(1)), 'des_pitch': float(m.group(2)), 'des_yaw': float(m.group(3))})
        return False

    m = PATTERNS['control'].search(line)
    if m:
        packet.update({'throttle': float(m.group(1)), 'des_throttle': float(m.group(2)), 'airspeed': float(m.group(3))})
        return False

    m = PATTERNS['altitude'].search(line)
    if m:
        packet.update({'altitude': float(m.group(1)), 'baro_altitude': float(m.group(2)), 'des_altitude': float(m.group(3))})
        return False

    m = PATTERNS['gps'].search(line)
    if m:
        packet.update({
            'gps_lat':     float(m.group(1)),
            'gps_long':    float(m.group(2)),
            'gps_alt':     float(m.group(3)),
            'gps_speed':   float(m.group(4)),
            'gps_heading': float(m.group(5)),
        })
        return False

    m = PATTERNS['wp'].search(line)
    if m:
        packet.update({
            'waypoint_distance':         float(m.group(1)),
            'waypoint_heading':          float(m.group(2)),
            'waypoint_index':            int(m.group(3)),
            'waypoint_total':            int(m.group(4)),
            'waypoint_mission_complete': int(m.group(5)),
        })
        return False

    m = PATTERNS['wptarget'].search(line)
    if m:
        packet.update({
            'waypoint_target_lat':       float(m.group(1)),
            'waypoint_target_lon':       float(m.group(2)),
            'waypoint_target_alt':       float(m.group(3)),
            'waypoint_leg_progress':     float(m.group(4)),
            'waypoint_mission_progress': float(m.group(5)),
        })
        return False

    m = PATTERNS['failsafe'].search(line)
    if m:
        packet.update({'failsafe_status': float(m.group(1))})
        return False

    m = PATTERNS['active_pid'].search(line)
    if m:
        packet.update({
            'roll_pid_kp': float(m.group(1)), 'roll_pid_ki': float(m.group(2)), 'roll_pid_kd': float(m.group(3)),
            'pitch_pid_kp': float(m.group(4)), 'pitch_pid_ki': float(m.group(5)), 'pitch_pid_kd': float(m.group(6)),
            'yaw_pid_kp': float(m.group(7)), 'yaw_pid_ki': float(m.group(8)), 'yaw_pid_kd': float(m.group(9)),
        })
        return False

    m = PATTERNS['nav_pid'].search(line)
    if m:
        packet.update({
            'altitude_pid_kp': float(m.group(1)), 'altitude_pid_ki': float(m.group(2)), 'altitude_pid_kd': float(m.group(3)),
            'headingerror_pid_kp': float(m.group(4)), 'headingerror_pid_ki': float(m.group(5)), 'headingerror_pid_kd': float(m.group(6)),
        })
        return False

    return False


def make_status_payload(ok: bool, *, sent: str = '', error: str = '') -> str:
    payload = {
        'type': 'command_status',
        'ok': ok,
        'ts': time.time(),
    }
    if sent:
        payload['sent'] = sent
    if error:
        payload['error'] = error
    return json.dumps(payload)


def extract_command(message: str) -> str:
    """
    Extract a command JSON object from a WebSocket message.

    Accepted message shapes:
    1) {"type":"command","payload":{...}}
    2) {"type":"command","command":"{...}"}
    3) {...}  (treated as a direct command object)
    """
    try:
        decoded = json.loads(message)
    except json.JSONDecodeError as exc:
        raise ValueError(f'Message is not valid JSON: {exc.msg}') from exc

    if not isinstance(decoded, dict):
        raise ValueError('Command message must be a JSON object.')

    if decoded.get('type') == 'command':
        if 'payload' in decoded:
            payload = decoded['payload']
            if not isinstance(payload, dict):
                raise ValueError("'payload' must be a JSON object.")
            return json.dumps(payload, separators=(',', ':'))

        raw = decoded.get('command')
        if not isinstance(raw, str):
            raise ValueError("Provide 'payload' object or 'command' JSON string.")
        return raw

    return json.dumps(decoded, separators=(',', ':'))


async def send_serial_command(command: str) -> str:
    """
    Validate and send one JSON command line to the receiver over serial.
    """
    global SERIAL_CONN

    command = command.strip()
    if not command:
        raise ValueError('Command is empty.')

    # Send the command as a raw UTF-8 string, because arduino_lora_receiver
    # expects plain text lines like "pid roll 20 0.1 0.05", not JSON.
    encoded = command.encode('utf-8')
    if len(encoded) > MAX_COMMAND_BYTES:
        raise ValueError(
            f'Command too long ({len(encoded)} > {MAX_COMMAND_BYTES} bytes).'
        )

    conn = SERIAL_CONN
    if conn is None or not conn.is_open:
        raise RuntimeError('Serial port is not open yet.')

    loop = asyncio.get_running_loop()
    payload = encoded + b'\n'

    async with SERIAL_WRITE_LOCK:
        await loop.run_in_executor(None, conn.write, payload)
        await loop.run_in_executor(None, conn.flush)

    return command


async def ws_handler(websocket):
    CLIENTS.add(websocket)
    print(f"[WS] Client connected ({websocket.remote_address}). Total: {len(CLIENTS)}")
    try:
        async for message in websocket:
            if not isinstance(message, str):
                await websocket.send(
                    make_status_payload(False, error='Binary WebSocket frames are not supported.')
                )
                continue

            try:
                command = extract_command(message)
                sent = await send_serial_command(command)
                print(f"\n[TX] JSON command: {sent}")
                await websocket.send(make_status_payload(True, sent=sent))
            except Exception as exc:
                await websocket.send(make_status_payload(False, error=str(exc)))
    finally:
        CLIENTS.discard(websocket)
        print(f"[WS] Client disconnected. Total: {len(CLIENTS)}")


async def broadcast(payload: str):
    if not CLIENTS:
        return

    clients = list(CLIENTS)
    results = await asyncio.gather(*[c.send(payload) for c in clients], return_exceptions=True)
    for client, result in zip(clients, results):
        if isinstance(result, Exception):
            CLIENTS.discard(client)


async def serial_reader(port: str, baud: int):
    global SERIAL_CONN

    loop = asyncio.get_running_loop()

    while True:
        ser = None
        current: dict = {}

        try:
            print(f"[Serial] Opening {port} at {baud} baud ...")
            ser = serial.Serial(port, baud, timeout=1)
            SERIAL_CONN = ser
            print("[Serial] Port open. Receiving telemetry ...\n")

            while True:
                raw = await loop.run_in_executor(None, ser.readline)
                line = raw.decode('utf-8', errors='replace').strip()
                
                # An empty line signals the end of the current packet group block.
                if not line:
                    if current:
                        await broadcast(json.dumps(current))
                        print(
                            f"\r[Serial] #{current.get('packet_id','?'):>5}  "
                            f"RSSI {current.get('rssi','?'):>4} dBm  "
                            f"SNR {current.get('snr', 0):>5.1f} dB  "
                            f"clients: {len(CLIENTS)}   ",
                            end='', flush=True,
                        )
                        current = {}
                    continue

                # Instantly broadcast LoRa PID command status logs to the UI
                if line.startswith('[TX]') or line.startswith('[ACK]') or line.startswith('Queued') or line.startswith('Cancelled') or line.startswith('Still waiting'):
                    await broadcast(json.dumps({'type': 'cmd_log', 'msg': line}))
                    print(f"\n{line}")
                    continue

                parse_line(line, current)

        except serial.SerialException as exc:
            print(f"\n[Serial] Port error: {exc}. Retrying in 2 s ...")
        except Exception as exc:
            print(f"\n[Serial] Error: {exc}. Retrying in 2 s ...")
        finally:
            SERIAL_CONN = None
            if ser is not None and ser.is_open:
                try:
                    ser.close()
                except serial.SerialException:
                    pass

        await asyncio.sleep(2)


async def main(args):
    if args.list_ports:
        ports = serial.tools.list_ports.comports()
        print("Available serial ports:")
        for p in (ports or []):
            print(f"  {p.device:<20} {p.description}")
        if not ports:
            print("  (none found)")
        return

    ws_server = await serve(ws_handler, "0.0.0.0", args.ws_port)
    print(f"[WS] WebSocket server listening on ws://0.0.0.0:{args.ws_port}")
    print(f"     Connect the GCS dashboard to: ws://localhost:{args.ws_port}\n")
    print('     Open gcs_gui.html to view telemetry and send command examples.\n')

    await asyncio.gather(
        serial_reader(args.port, args.baud),
        ws_server.wait_closed(),
    )


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='GCS serial-to-WebSocket telemetry bridge')
    ap.add_argument('--port',       default='/dev/ttyUSB0',
                    help='Serial port (e.g. /dev/ttyUSB0  or  COM5)')
    ap.add_argument('--baud',       type=int, default=921600,
                    help='Baud rate (default 921600)')
    ap.add_argument('--ws-port',    type=int, default=8765,
                    help='WebSocket listen port (default 8765)')
    ap.add_argument('--list-ports', action='store_true',
                    help='List available serial ports and exit')
    asyncio.run(main(ap.parse_args()))
