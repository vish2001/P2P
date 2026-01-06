import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# ==========================================
# 1. THE ONLY INPUT: DISTANCE LIST
# ==========================================
# I am manually specifying these. 
# There is no "True Coordinates" variable anywhere in this script.
# We represent a 3x4 Rectangle with diagonals of length 5.

N_NODES = 4

# Dictionary format: (Node_A, Node_B) : Length
known_distances = {
    (0, 1): 3.0,  # Bottom Edge
    (1, 2): 4.0,  # Right Edge
    (2, 3): 3.0,  # Top Edge
    (3, 0): 4.0,  # Left Edge
    (0, 2): 5.0,  # Diagonal 1 (Hypotenuse)
    (1, 3): 5.0   # Diagonal 2 (Hypotenuse)
}

# ==========================================
# 2. THE SOLVER
# ==========================================
# The computer has to invent coordinates (x,y) that make these distances true.

def stress_function(flat_coords):
    """
    Input: A flat list of random coordinates [x0, y0, x1, y1...]
    Output: How 'bad' the shape is (Error)
    """
    # Reshape into x,y pairs
    points = flat_coords.reshape((N_NODES, 2))
    
    total_error = 0
    
    # Loop through our ONLY input data (the distance list)
    for (u, v), target_length in known_distances.items():
        
        # Calculate how far apart the solver *thinks* u and v are right now
        current_dist = np.linalg.norm(points[u] - points[v])
        
        # We want the difference to be 0
        total_error += (current_dist - target_length)**2
        
    return total_error

# ==========================================
# 3. EXECUTION
# ==========================================

print("Starting Solver...")
print("Input Data:", known_distances)

# Start with complete junk (Random Noise)
# This proves we aren't using pre-existing locations.
np.random.seed(42)
initial_guess = np.random.rand(N_NODES * 2)

# Run Optimization
# We assume Node 0 is at (0,0) to pin the map down, otherwise it floats away.
# This doesn't set the shape, just the location on the plot.
cons = ({'type': 'eq', 'fun': lambda x: x[0]}, # x0 = 0
        {'type': 'eq', 'fun': lambda x: x[1]}) # y0 = 0

res = minimize(stress_function, initial_guess, constraints=cons, method='SLSQP', tol=1e-8)

# Extract the discovered coordinates
discovered_coords = res.x.reshape((N_NODES, 2))

print("\nSolution Found!")
print(f"Final Error (Stress): {res.fun:.10f} (Should be close to 0)")

# ==========================================
# 4. VISUALIZATION
# ==========================================
plt.figure(figsize=(6, 6))
plt.title("Shape Discovered from Pure Distances")

# Plot Nodes
plt.scatter(discovered_coords[:,0], discovered_coords[:,1], s=200, color='red', zorder=5)

# Label Nodes
for i in range(N_NODES):
    plt.text(discovered_coords[i,0]+0.1, discovered_coords[i,1]+0.1, f"Node {i}", fontsize=12)

# Plot Edges
for (u, v), length in known_distances.items():
    p1 = discovered_coords[u]
    p2 = discovered_coords[v]
    
    # Draw line
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k--', alpha=0.5)
    
    # Label line with the distance provided in input
    mid = (p1 + p2) / 2
    plt.text(mid[0], mid[1], f"{length}", color='blue', backgroundcolor='white')

plt.axis('equal')
plt.grid(True)
plt.show()

print("\n--- The Calculated Coordinates ---")
print("These were created by the math solely to satisfy your distances.")
for i in range(N_NODES):
    print(f"Node {i}: {discovered_coords[i]}")