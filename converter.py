import open3d as o3d
import sys
import os
import argparse

def convert_point_cloud(input_file, output_file):
    # Check if input file exists
    if not os.path.isfile(input_file):
        print(f"Error: Input file {input_file} does not exist.")
        return False

    # Check file extensions
    input_ext = os.path.splitext(input_file)[1].lower()
    output_ext = os.path.splitext(output_file)[1].lower()
    
    if input_ext not in ['.ply', '.pcd'] or output_ext not in ['.ply', '.pcd']:
        print("Error: Only .ply and .pcd formats are supported.")
        return False

    # Read the point cloud
    print(f"Reading file: {input_file}")
    try:
        point_cloud = o3d.io.read_point_cloud(input_file)
    except Exception as e:
        print(f"Error reading file: {str(e)}")
        return False

    if not point_cloud.has_points():
        print("Error: The point cloud is empty or failed to load.")
        return False

    # Save in new format
    print(f"Converting to: {output_file}")
    try:
        o3d.io.write_point_cloud(output_file, point_cloud)
        print("Conversion successful!")
        return True
    except Exception as e:
        print(f"Error writing file: {str(e)}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Convert between PLY and PCD point cloud formats.')
    parser.add_argument('input_file', help='Input point cloud file (.ply or .pcd)')
    parser.add_argument('output_file', help='Output point cloud file (.ply or .pcd)')
    args = parser.parse_args()

    convert_point_cloud(args.input_file, args.output_file)

if __name__ == "__main__":
    main()