import socket
import json
import time
import threading
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Known anchor positions (in meters)
ANCHORS = {
    "E8:6B:EA:D4:30:34": (0.0, 0.0),  # Anchor A
    "EC:64:C9:85:72:C0": (0.0, 15.0),  # Anchor B
    "0C:8B:95:76:AD:20": (5.0, 0.0),   # Anchor C
    "0C:8B:95:76:1F:00": (15.0, 5.0)  # Anchor D

}
positions = {}
# Path-loss model parameters
A = -45.0  # RSSI at 1 meter (calibrate for your environment)
n = 2.0    # path-loss exponent

# UDP listener configuration
UDP_IP = '192.168.0.204'   # listen only on this PC's IP
UDP_PORT = 5005
BUFFER_TIME = 1.0  # seconds window

# Thread-safe buffer to store incoming readings
data_buffer = []
buffer_lock = threading.Lock()

# Store last known trilaterated positions and their timestamps
last_known_positions = {}  # {target: (pos, timestamp)}

# Listen for incoming UDP packets and append to buffer
def listen_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening on UDP {UDP_IP}:{UDP_PORT}")
    while True:
        data, _ = sock.recvfrom(1024)
        try:
            msg = json.loads(data.decode())
            timestamp = time.time()
            anchor = msg.get('anchor')
            target = msg.get('target')
            rssi = msg.get('rssi')
            # Normalize MAC addresses to uppercase
            if anchor: anchor = anchor.upper()
            if target: target = target.upper()
            print(f"Received anchor: {anchor}, target: {target}, rssi: {rssi}")  # Debug print
            with buffer_lock:
                data_buffer.append((timestamp, anchor, target, rssi))
        except Exception as e:
            print(f"Error parsing packet: {e}")

# Convert RSSI to estimated distance
def rssi_to_distance(rssi):
    return 10 ** ((A - rssi) / (10 * n))

# Trilateration using three anchors
def trilaterate(distances):
    keys = list(distances.keys())
    x1, y1 = ANCHORS[keys[0]]; d1 = distances[keys[0]]
    x2, y2 = ANCHORS[keys[1]]; d2 = distances[keys[1]]
    x3, y3 = ANCHORS[keys[2]]; d3 = distances[keys[2]]
    Acoef = 2 * (x2 - x1)
    Bcoef = 2 * (y2 - y1)
    Ccoef = d1**2 - d2**2 - x1**2 + x2**2 - y1**2 + y2**2
    Dcoef = 2 * (x3 - x1)
    Ecoef = 2 * (y3 - y1)
    Fcoef = d1**2 - d3**2 - x1**2 + x3**2 - y1**2 + y3**2
    denom = Acoef * Ecoef - Bcoef * Dcoef
    if abs(denom) < 1e-6:
        return None
    x = (Ccoef * Ecoef - Bcoef * Fcoef) / denom
    y = (Acoef * Fcoef - Ccoef * Dcoef) / denom
    return (x, y)

# Main loop to process buffered data and compute position
def process_data():
    plt.ion()
    fig, ax = plt.subplots()
    last_update = 0
    while True:
        time.sleep(BUFFER_TIME)
        cutoff = time.time() - BUFFER_TIME
        now = time.time()
        with buffer_lock:
            recent = [(anchor, target, rssi) for (ts, anchor, target, rssi) in data_buffer if ts >= cutoff]
            data_buffer[:] = [(ts, anchor, target, rssi) for (ts, anchor, target, rssi) in data_buffer if ts >= cutoff]
        readings = {}
        for anchor, target, rssi in recent:
            if target not in readings:
                readings[target] = {}
            readings[target][anchor] = max(readings[target].get(anchor, -999), rssi)
        updated_this_cycle = set()
        if now - last_update >= 10:
            ax.clear()
            # Plot anchors again after clearing
            for mac, (x, y) in ANCHORS.items():
                ax.plot(x, y, 'bs', label='Anchor' if mac == list(ANCHORS.keys())[0] else "")
                ax.text(x, y, mac, fontsize=8, color='blue')
            count_trilat = 0
            count_approx = 0
            # Update last known positions if new trilateration
            for target, best_rssi in readings.items():
                anchors_present = [anchor for anchor in best_rssi if anchor in ANCHORS]
                if target in ANCHORS:
                    continue
                if len(anchors_present) >= 3:
                    distances = {anchor: rssi_to_distance(best_rssi[anchor]) for anchor in anchors_present}
                    pos = trilaterate(distances)
                    if pos is not None and not (math.isnan(pos[0]) or math.isnan(pos[1])):
                        last_known_positions[target] = (pos, now)
                        ax.plot(pos[0], pos[1], 'ro', label='Trilaterated' if count_trilat == 0 else "")
                        ax.text(pos[0], pos[1], target, fontsize=8, color='red')
                        count_trilat += 1
                        updated_this_cycle.add(target)
                elif len(anchors_present) >= 1:
                    anchor = max(anchors_present, key=lambda a: best_rssi[a])
                    x, y = ANCHORS[anchor]
                    ax.plot(x, y, 'y^', label='Approximate' if count_approx == 0 else "")
                    ax.text(x, y, target, fontsize=8, color='orange')
                    count_approx += 1
                    updated_this_cycle.add(target)
            # Only plot last known positions for MACs not updated this cycle
            for target, (pos, ts) in last_known_positions.items():
                if target not in updated_this_cycle and now - ts < 10:
                    ax.plot(pos[0], pos[1], 'ro', alpha=0.4, label='Last known')
                    ax.text(pos[0], pos[1], f"{target}\n(last)", fontsize=8, color='red', alpha=0.4)
            ax.set_xlim(-1, 6)
            ax.set_ylim(-1, 16)
            ax.set_title(f"Targets: {count_trilat} trilaterated, {count_approx} approximate")
            ax.set_xlabel("X (meters)")
            ax.set_ylabel("Y (meters)")
            handles, labels = ax.get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            ax.legend(by_label.values(), by_label.keys())
            plt.pause(0.01)
            last_update = now
        else:
            # Still update last_known_positions if new trilateration, but don't update the plot
            for target, best_rssi in readings.items():
                anchors_present = [anchor for anchor in best_rssi if anchor in ANCHORS]
                if target in ANCHORS:
                    continue
                if len(anchors_present) >= 3:
                    distances = {anchor: rssi_to_distance(best_rssi[anchor]) for anchor in anchors_present}
                    pos = trilaterate(distances)
                    if pos is not None and not (math.isnan(pos[0]) or math.isnan(pos[1])):
                        last_known_positions[target] = (pos, now)

if __name__ == '__main__':
    listener = threading.Thread(target=listen_udp, daemon=True)
    listener.start()
    process_data()