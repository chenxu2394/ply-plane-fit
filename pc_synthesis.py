import open3d as o3d
import numpy as np
import random

# Ground dimensions and grid setup
ground_size = 10
grid_cells = 2  # 2x2 grid
cell_size = ground_size / grid_cells
min_distance = 2.0  # Minimum distance between box centers

def get_random_position(cell_x, cell_z):
    # Get position within specific cell
    x_min = -ground_size/2 + cell_x * cell_size
    x_max = x_min + cell_size
    z_min = -ground_size/2 + cell_z * cell_size
    z_max = z_min + cell_size
    
    x = random.uniform(x_min + 1, x_max - 2)
    z = random.uniform(z_min + 1, z_max - 2)
    return x, z

# Create list to store box positions
box_positions = []

# Generate boxes in grid cells
for i in range(grid_cells):
    for j in range(grid_cells):
        width = random.uniform(0.5, 2.0)
        height = random.uniform(0.5, 3.0)
        depth = random.uniform(0.5, 2.0)
        
        x, z = get_random_position(i, j)
        box_positions.append((x, z, width, height, depth))

# Create randomized ground plane points
num_ground_points = 10000  # Number of points for ground
ground_points = np.zeros((num_ground_points, 3))

# Random X and Z coordinates within ground boundaries
ground_points[:, 0] = np.random.uniform(-ground_size/2, ground_size/2, num_ground_points)  # X
ground_points[:, 2] = np.random.uniform(-ground_size/2, ground_size/2, num_ground_points)  # Z

# Add small random noise to Y coordinate (optional)
ground_points[:, 1] = np.random.normal(0, 0.02, num_ground_points)  # Y with small noise

# Convert to point cloud
ground_pcd = o3d.geometry.PointCloud()
ground_pcd.points = o3d.utility.Vector3dVector(ground_points)
ground_pcd.paint_uniform_color([0.8, 0.8, 0.8])

# Convert to point cloud
ground_pcd = o3d.geometry.PointCloud()
ground_pcd.points = o3d.utility.Vector3dVector(ground_points)
ground_pcd.paint_uniform_color([0.8, 0.8, 0.8])  # Light gray color

# Generate multiple boxes
all_pcds = []
# Create boxes using stored positions
all_pcds = []
for x, z, width, height, depth in box_positions:
    box = o3d.geometry.TriangleMesh.create_box(width=width, height=height, depth=depth)
    box.translate(np.array([x, 0, z]))
    pcd = box.sample_points_poisson_disk(number_of_points=1000)
    pcd.paint_uniform_color([0.1, 0.1, 1])
    all_pcds.append(pcd)

# Combine all point clouds including ground
combined_pcd = ground_pcd
for pcd in all_pcds:
    combined_pcd += pcd

o3d.visualization.draw_geometries([combined_pcd])