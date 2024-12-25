import open3d as o3d
import os

# Get absolute path to the bag file
bag_file = "../20241204_094513.bag"
abs_bag_file = os.path.abspath(bag_file)

try:
    # Initialize and open bag reader
    bag_reader = o3d.t.io.RSBagReader()
    
    if not os.path.exists(abs_bag_file):
        raise FileNotFoundError(f"Bag file not found: {abs_bag_file}")
        
    bag_reader.open(abs_bag_file)
    
    im_rgbd = bag_reader.next_frame()
    while not bag_reader.is_eof():
        # process im_rgbd.depth and im_rgbd.color
        im_rgbd = bag_reader.next_frame()

except Exception as e:
    print(f"Error processing bag file: {e}")
finally:
    # Ensure reader is closed even if error occurs
    if 'bag_reader' in locals():
        bag_reader.close()