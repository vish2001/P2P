#!/usr/bin/env python3
"""
UWB P2P Mesh Localizer - Central Station Receiver

Receives pairwise distance measurements from ESP32 nodes via WiFi UDP,
performs MDS (Multidimensional Scaling) localization, and visualizes
the mesh network in real-time.

Usage:
    python uwb_mesh_localizer.py

Requirements:
    pip install numpy matplotlib scipy

Protocol:
    R,<from_id>,<to_id>,<distance_cm>,<rssi>,<timestamp>  - Ranging result
    N,<node_id>,<neighbor_id>,<hello_count>,<range_pct>,<dist>,<rssi>  - Neighbor info
    H,<node_id>,<frame_num>,<neighbor_count>,<uptime_ms>  - Heartbeat
"""

import socket
import threading
import time
import numpy as np
from collections import defaultdict
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.distance import squareform
from scipy.linalg import eigh

# =============================================================================
# CONFIGURATION
# =============================================================================
UDP_PORT = 5000
DISTANCE_EXPIRY_SEC = 10.0  # Distance measurements expire after this time
UPDATE_INTERVAL_MS = 500    # Visualization update interval
MAX_NODES = 10              # Maximum number of nodes to track

# =============================================================================
# DATA STRUCTURES
# =============================================================================
class DistanceMatrix:
    """Thread-safe distance matrix with timestamps"""
    
    def __init__(self):
        self.distances = {}  # (from_id, to_id) -> (distance_cm, timestamp, rssi)
        self.nodes = set()
        self.lock = threading.Lock()
        
    def update(self, from_id, to_id, distance_cm, rssi=None):
        """Update distance between two nodes"""
        with self.lock:
            # Store bidirectional (symmetric)
            key1 = (min(from_id, to_id), max(from_id, to_id))
            self.distances[key1] = (distance_cm, time.time(), rssi)
            self.nodes.add(from_id)
            self.nodes.add(to_id)
    
    def get_matrix(self):
        """Get distance matrix for all known nodes, pruning expired entries"""
        with self.lock:
            now = time.time()
            
            # Prune expired entries
            expired = [k for k, v in self.distances.items() 
                      if now - v[1] > DISTANCE_EXPIRY_SEC]
            for k in expired:
                del self.distances[k]
            
            # Get sorted node list
            node_list = sorted(self.nodes)
            n = len(node_list)
            
            if n < 2:
                return None, node_list
            
            # Build distance matrix
            D = np.zeros((n, n))
            for i, node_i in enumerate(node_list):
                for j, node_j in enumerate(node_list):
                    if i == j:
                        continue
                    key = (min(node_i, node_j), max(node_i, node_j))
                    if key in self.distances:
                        D[i, j] = self.distances[key][0]
                    else:
                        D[i, j] = np.nan  # Missing measurement
            
            return D, node_list
    
    def get_all_distances(self):
        """Get list of all current distances for display"""
        with self.lock:
            now = time.time()
            result = []
            for (from_id, to_id), (dist, ts, rssi) in self.distances.items():
                age = now - ts
                if age < DISTANCE_EXPIRY_SEC:
                    result.append({
                        'from': from_id,
                        'to': to_id,
                        'distance': dist,
                        'rssi': rssi,
                        'age': age
                    })
            return result

# =============================================================================
# MDS LOCALIZATION
# =============================================================================
def mds_localization(D, n_components=2):
    """
    Perform classical MDS (Multidimensional Scaling) localization.
    
    Given a distance matrix D, estimate 2D positions of nodes.
    
    Args:
        D: NxN distance matrix (cm). NaN values are treated as missing.
        n_components: Number of dimensions (2 for 2D visualization)
    
    Returns:
        positions: Nx2 array of estimated positions, or None if failed
    """
    n = D.shape[0]
    if n < 3:
        # Can't localize with fewer than 3 nodes
        if n == 2:
            # Just place them on a line
            d = np.nanmean([D[0,1], D[1,0]])
            if np.isnan(d):
                d = 100  # Default
            return np.array([[0, 0], [d, 0]])
        return None
    
    # Handle missing values by using mean distance
    D_filled = D.copy()
    mean_dist = np.nanmean(D[D > 0])
    if np.isnan(mean_dist):
        mean_dist = 100  # Default
    D_filled[np.isnan(D_filled)] = mean_dist
    
    # Make symmetric
    D_sym = (D_filled + D_filled.T) / 2
    np.fill_diagonal(D_sym, 0)
    
    # Classical MDS
    # 1. Square the distances
    D2 = D_sym ** 2
    
    # 2. Double centering
    n = D2.shape[0]
    H = np.eye(n) - np.ones((n, n)) / n
    B = -0.5 * H @ D2 @ H
    
    # 3. Eigen decomposition
    try:
        eigenvalues, eigenvectors = eigh(B)
        
        # Sort by eigenvalue (descending)
        idx = np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        # Take top n_components
        # Handle negative eigenvalues (can happen with noisy data)
        eigenvalues = np.maximum(eigenvalues[:n_components], 0)
        
        # Compute positions
        positions = eigenvectors[:, :n_components] * np.sqrt(eigenvalues)
        
        return positions
        
    except Exception as e:
        print(f"MDS failed: {e}")
        return None

# =============================================================================
# UDP RECEIVER
# =============================================================================
class UDPReceiver:
    """Receives UDP packets from ESP32 nodes"""
    
    def __init__(self, port, distance_matrix):
        self.port = port
        self.dm = distance_matrix
        self.running = False
        self.packet_count = 0
        self.last_packet_time = 0
        
    def start(self):
        """Start receiver thread"""
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        print(f"[UDP] Listening on port {self.port}")
        
    def stop(self):
        """Stop receiver"""
        self.running = False
        
    def _receive_loop(self):
        """Main receive loop"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', self.port))
        sock.settimeout(1.0)
        
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                self._process_packet(data.decode('utf-8'), addr)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[UDP] Error: {e}")
                
        sock.close()
        
    def _process_packet(self, data, addr):
        """Process received packet"""
        self.packet_count += 1
        self.last_packet_time = time.time()
        
        try:
            parts = data.strip().split(',')
            msg_type = parts[0]
            
            if msg_type == 'R':  # Ranging result
                from_id = int(parts[1])
                to_id = int(parts[2])
                distance_cm = float(parts[3])
                rssi = float(parts[4]) if len(parts) > 4 else None
                
                self.dm.update(from_id, to_id, distance_cm, rssi)
                
                print(f"[RX] {from_id} -> {to_id}: {distance_cm:.1f} cm "
                      f"(RSSI: {rssi:.1f} dBm)" if rssi else "")
                
            elif msg_type == 'H':  # Heartbeat
                node_id = int(parts[1])
                frame_num = int(parts[2])
                neighbor_count = int(parts[3])
                print(f"[HB] Node {node_id}: frame={frame_num}, neighbors={neighbor_count}")
                
            elif msg_type == 'N':  # Neighbor info
                # Just log it
                pass
                
        except Exception as e:
            print(f"[UDP] Parse error: {e} - Data: {data}")

# =============================================================================
# VISUALIZATION
# =============================================================================
class MeshVisualizer:
    """Real-time mesh visualization using matplotlib"""
    
    def __init__(self, distance_matrix):
        self.dm = distance_matrix
        self.fig, (self.ax_mesh, self.ax_table) = plt.subplots(1, 2, figsize=(14, 7))
        self.fig.suptitle('UWB P2P Mesh Localization', fontsize=14)
        
        # Colors for nodes
        self.colors = plt.cm.tab10(np.linspace(0, 1, MAX_NODES))
        
    def update(self, frame):
        """Update visualization"""
        self.ax_mesh.clear()
        self.ax_table.clear()
        
        # Get distance matrix
        D, node_list = self.dm.get_matrix()
        
        if D is None or len(node_list) < 2:
            self.ax_mesh.text(0.5, 0.5, 'Waiting for data...', 
                            ha='center', va='center', fontsize=14)
            self.ax_mesh.set_xlim(-1, 1)
            self.ax_mesh.set_ylim(-1, 1)
            return
        
        # Perform MDS localization
        positions = mds_localization(D)
        
        if positions is None:
            self.ax_mesh.text(0.5, 0.5, 'Localization failed', 
                            ha='center', va='center', fontsize=14)
            return
        
        # Plot mesh
        self._plot_mesh(positions, node_list, D)
        
        # Plot distance table
        self._plot_table()
        
    def _plot_mesh(self, positions, node_list, D):
        """Plot the mesh network"""
        n = len(node_list)
        
        # Draw edges (connections between nodes)
        for i in range(n):
            for j in range(i+1, n):
                if not np.isnan(D[i, j]) and D[i, j] > 0:
                    x = [positions[i, 0], positions[j, 0]]
                    y = [positions[i, 1], positions[j, 1]]
                    
                    # Color based on distance
                    dist = D[i, j]
                    self.ax_mesh.plot(x, y, 'gray', alpha=0.5, linewidth=1)
                    
                    # Label with distance
                    mid_x = (x[0] + x[1]) / 2
                    mid_y = (y[0] + y[1]) / 2
                    self.ax_mesh.text(mid_x, mid_y, f'{dist:.0f}', 
                                     fontsize=8, ha='center', va='center',
                                     bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))
        
        # Draw nodes
        for i, node_id in enumerate(node_list):
            color = self.colors[node_id % MAX_NODES]
            self.ax_mesh.scatter(positions[i, 0], positions[i, 1], 
                               s=500, c=[color], edgecolors='black', linewidths=2, zorder=5)
            self.ax_mesh.text(positions[i, 0], positions[i, 1], str(node_id),
                            fontsize=12, ha='center', va='center', fontweight='bold', zorder=6)
        
        # Formatting
        self.ax_mesh.set_xlabel('X (cm)')
        self.ax_mesh.set_ylabel('Y (cm)')
        self.ax_mesh.set_title(f'Mesh Topology ({n} nodes)')
        self.ax_mesh.set_aspect('equal')
        self.ax_mesh.grid(True, alpha=0.3)
        
        # Add scale
        margin = 50
        xlim = self.ax_mesh.get_xlim()
        ylim = self.ax_mesh.get_ylim()
        self.ax_mesh.set_xlim(xlim[0] - margin, xlim[1] + margin)
        self.ax_mesh.set_ylim(ylim[0] - margin, ylim[1] + margin)
        
    def _plot_table(self):
        """Plot distance table"""
        distances = self.dm.get_all_distances()
        
        if not distances:
            self.ax_table.text(0.5, 0.5, 'No distances', ha='center', va='center')
            self.ax_table.set_xlim(0, 1)
            self.ax_table.set_ylim(0, 1)
            return
        
        # Sort by from_id, then to_id
        distances.sort(key=lambda x: (x['from'], x['to']))
        
        # Create table data
        columns = ['From', 'To', 'Distance (cm)', 'RSSI (dBm)', 'Age (s)']
        cell_text = []
        for d in distances:
            cell_text.append([
                str(d['from']),
                str(d['to']),
                f"{d['distance']:.1f}",
                f"{d['rssi']:.1f}" if d['rssi'] else 'N/A',
                f"{d['age']:.1f}"
            ])
        
        # Draw table
        self.ax_table.axis('off')
        table = self.ax_table.table(cellText=cell_text, colLabels=columns,
                                    loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1.2, 1.5)
        
        self.ax_table.set_title(f'Pairwise Distances ({len(distances)} links)')
        
    def run(self):
        """Start visualization"""
        ani = FuncAnimation(self.fig, self.update, interval=UPDATE_INTERVAL_MS, cache_frame_data=False)
        plt.tight_layout()
        plt.show()

# =============================================================================
# MAIN
# =============================================================================
def main():
    print("=" * 60)
    print("  UWB P2P Mesh Localizer - Central Station")
    print("=" * 60)
    print(f"Listening on UDP port {UDP_PORT}")
    print("Waiting for data from ESP32 nodes...")
    print("=" * 60)
    
    # Create distance matrix
    dm = DistanceMatrix()
    
    # Start UDP receiver
    receiver = UDPReceiver(UDP_PORT, dm)
    receiver.start()
    
    # Start visualization
    viz = MeshVisualizer(dm)
    
    try:
        viz.run()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        receiver.stop()

if __name__ == '__main__':
    main()


