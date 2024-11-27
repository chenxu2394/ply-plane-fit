import open3d as o3d
import sys
import os
import argparse

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='View PLY point cloud file.')
    parser.add_argument('ply_file', type=str, help='Path to the PLY file')
    args = parser.parse_args()

    ply_file = args.ply_file

    # Check if the file exists
    if not os.path.isfile(ply_file):
        print(f"Error: The file {ply_file} does not exist.")
        sys.exit(1)

    # Read the point cloud from the PLY file
    print(f"Reading PLY file: {ply_file}")
    point_cloud = o3d.io.read_point_cloud(ply_file)

    # Check if the point cloud was successfully loaded
    if not point_cloud.has_points():
        print("Error: The point cloud is empty or failed to load.")
        sys.exit(1)

    # Print some information about the point cloud
    print(f"Point cloud successfully loaded:")
    print(f" - Number of points: {len(point_cloud.points)}")
    if point_cloud.has_colors():
        print(" - Point cloud has color information.")
    if point_cloud.has_normals():
        print(" - Point cloud has normal vectors.")

    # Visualize the point cloud
    print("Visualizing the point cloud...")
    o3d.visualization.draw_geometries([point_cloud],
                                    window_name='Open3D - PLY Viewer',
                                    width=800,
                                    height=600,
                                    left=50,
                                    top=50,
                                    point_show_normal=False)

if __name__ == "__main__":
    main()
