import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("data/8.ply")

# Detect planar patches
oboxes = pcd.detect_planar_patches(
    normal_variance_threshold_deg=60,
    coplanarity_deg=85,
    outlier_ratio=0.75,
    min_plane_edge_length=0,
    min_num_points=0,
    search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

print("Detected {} patches".format(len(oboxes)))

# Keep only first 3 planes
oboxes = oboxes[0:3]

# Get points that belong to these 3 planes
points = np.asarray(pcd.points)
keep_indices = []

geometries = []

for obox in oboxes:
    # Transform points to local coordinate system of the oriented box
    points_local = np.asarray(points) - np.asarray(obox.center)
    points_local = np.dot(points_local, np.asarray(obox.R))
    
    # Check which points are within the box bounds
    half_extent = np.asarray(obox.extent) / 2
    mask = np.all(np.abs(points_local) <= half_extent, axis=1)
    keep_indices.extend(np.where(mask)[0])

    mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.0001])
    mesh.paint_uniform_color(obox.color)
    geometries.append(mesh)
    geometries.append(obox)

# Remove duplicates from indices
keep_indices = list(set(keep_indices))

# Create new point cloud with only points from the 3 planes
filtered_pcd = pcd.select_by_index(keep_indices)

bounded_box = filtered_pcd.get_oriented_bounding_box()
bounded_box.color = (1, 0, 0)

# Instead of one draw_geometries call:
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add point cloud separately so we can specify its point size
vis.add_geometry(pcd)

# Add other geometries normally
vis.add_geometry(bounded_box)
for g in geometries:
    vis.add_geometry(g)

# Adjust render option
render_opt = vis.get_render_option()
render_opt.point_size = 1.8  # smaller points for pcd

vis.run()
vis.destroy_window()