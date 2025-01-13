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

# Define camera parameters
camera_height = 5.0  # Height of drone
camera_position = np.array([0, camera_height, -5])  # Drone position
camera_target = np.array([0, 0, 0])  # Looking at center of scene
camera_up = np.array([0, 1, 0])  # Up direction
fov = 60  # Field of view in degrees
aspect = 1.0
near = 0.1
far = 20

# # Create view and projection matrices
# view = np.array(o3d.camera.create_look_at(
#     camera_position,
#     camera_target,
#     camera_up
# ))

# Create pinhole camera parameters
intrinsic = o3d.camera.PinholeCameraIntrinsic()
intrinsic.set_intrinsics(
    width=640, height=480,
    fx=580, fy=580,
    cx=320, cy=240
)

# Create hidden point removal operator
radius = 2000
_, pt_map = combined_pcd.hidden_point_removal(camera_position, radius)

# Create new point cloud with only visible points
visible_pcd = combined_pcd.select_by_index(pt_map)

# # Visualize
# o3d.visualization.draw_geometries([visible_pcd])

# Visualize camera position
camera_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.2)
camera_sphere.translate(camera_position)
camera_sphere.paint_uniform_color([1, 0, 0])  # Red color for camera

# Visualize scene with camera position
o3d.visualization.draw_geometries([visible_pcd, camera_sphere])