import open3d as o3d

# dataset = o3d.data.PCDPointCloud()
# pcd = o3d.io.read_point_cloud(dataset.path)
# assert (pcd.has_normals())

pcd = o3d.io.read_point_cloud("data/8.ply")

# using all defaults
oboxes = pcd.detect_planar_patches(
    normal_variance_threshold_deg=60,
    coplanarity_deg=85,
    outlier_ratio=0.75,
    min_plane_edge_length=0,
    min_num_points=0,
    search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

print("Detected {} patches".format(len(oboxes)))

oboxes = oboxes[0:3]

geometries = []
for obox in oboxes:
    mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.0001])
    mesh.paint_uniform_color(obox.color)
    geometries.append(mesh)
    geometries.append(obox)
geometries.append(pcd)

# o3d.visualization.draw_geometries(geometries,
#                                   zoom=0.62,
#                                   front=[0.4361, -0.2632, -0.8605],
#                                   lookat=[2.4947, 1.7728, 1.5541],
#                                   up=[-0.1726, -0.9630, 0.2071])

o3d.visualization.draw_geometries(geometries)