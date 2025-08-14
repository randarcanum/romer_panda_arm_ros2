import pyrealsense2 as rs
import numpy as np
import pyigtl
import time
from scipy.ndimage import gaussian_filter
from scipy.spatial.transform import Rotation as R
import camera_cartesian as client

def stream_depth_volume(igtl_port=18944, device_name="DepthVolume"):
    """
    Streams depth data from RealSense camera, converts to 3D volume,
    and sends to 3D Slicer via OpenIGTLink with improved visual quality.
    """
    MOUNTED = True # True if the camera is mounted on the Panda robot arm
    print(">>Streaming depth volume to 3D Slicer<<")
    print("Coordinate system: Camera X=right, Y=down, Z=forward")
    print("Volume system: Volume X=right, Y=forward, Z=up")
    # Configure RealSense pipeline with optimized settings
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    if not MOUNTED:
        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    else:
        client.initialize()
    # Start pipeline and get depth scale
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Depth scale: {depth_scale:.6f} m/unit")
    # Extended depth range for better coverage (0.2m to 0.8m)
    MIN_DEPTH = 0.05
    MAX_DEPTH = 0.5
    min_depth_px = int(MIN_DEPTH / depth_scale)
    max_depth_px = int(MAX_DEPTH / depth_scale)
    print(f"Depth range: {MIN_DEPTH}-{MAX_DEPTH}m -> {min_depth_px}-{max_depth_px} depth units")
    # Start OpenIGTLink server
    server = pyigtl.OpenIGTLinkServer(port=igtl_port)
    print(f"OpenIGTLink server started on port {igtl_port}")
    # Optimized volume configuration for better visual quality
    GRID_SIZE = (320, 320, 240)  # Higher voxel density
    PHYSICAL_SIZE = (0.3, 0.3, 0.3)  # Smaller physical space
    COLOR = 200
    spacing = [PHYSICAL_SIZE[i] * 1000 / GRID_SIZE[i] for i in range(3)]  # Voxel spacing in mm
    print(f"Volume size: {GRID_SIZE} voxels, {PHYSICAL_SIZE} meters")
    print(f"Voxel spacing: {spacing} mm")
    # RealSense processing utilities
    pc = rs.pointcloud()
    # Tracking variables (using only rotation for better stability)
    ROTATION_INIT = R.from_quat([0, 1, 0, 0])
    TRANSLATION_INIT = np.array([0.55, 0.0, 0.4])
    rotation_quat = R.identity()  # Initial camera rotation
    translation_vec = np.zeros(3, dtype=np.double)
    prev_time = time.time()
    # Multi-frame array
    volume = np.zeros(GRID_SIZE, dtype=np.uint8)
    try:
        frame_count = 0
        while True:
            arm_stopped = False
            # Get frames from camera
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not MOUNTED:
                gyro_f = frames.first_or_default(rs.stream.gyro)

            # Calculate time delta for rotation tracking
            curr_time = time.time()
            dt = curr_time - prev_time
            prev_time = curr_time
            # Process gyroscope data for orientation tracking
            if MOUNTED:
                pose = client.get_cart()
                if pose[0]:
                    translation_vec = pose[1] - TRANSLATION_INIT
                    translation_vec[0], translation_vec[1] = translation_vec[1], translation_vec[0]
                    rotation_quat = ROTATION_INIT * pose[2]
                    arm_stopped = pose[3]
            else:
                if gyro_f:
                    gyro_data = gyro_f.as_motion_frame().get_motion_data()
                    angular_velocity = np.array([gyro_data.x, gyro_data.y, gyro_data.z])
                    rotation_quat = rotation_quat * R.from_rotvec(angular_velocity * dt)
            
            if not depth_frame or not arm_stopped:
                continue
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            # Generate point cloud and create depth mask
            points = pc.calculate(depth_frame)
            vtx = np.asanyarray(points.get_vertices())
            vtx = vtx.view(np.float32).reshape(480, 848, 3)  # Shape: (height, width, XYZ)
            
            # Create depth image and mask with extended range
            depth_image = np.asanyarray(depth_frame.get_data())
            mask = np.logical_and(depth_image > min_depth_px, depth_image < max_depth_px)
            # More lenient point threshold (5000 instead of 10000)
            valid_point_count = np.count_nonzero(mask)
            if valid_point_count < 5000:
                print(f"⚠️ Skipping frame: Only {valid_point_count} valid points")
                continue
            # Extract valid points and apply rotation only
            valid_pts = vtx[mask]
            valid_pts = (valid_pts @ rotation_quat.as_matrix().T) - translation_vec
            # Define volume bounds centered around camera
            min_bound = np.array([-PHYSICAL_SIZE[0]/2, -PHYSICAL_SIZE[1]/2, 0.2])
            max_bound = min_bound + np.array(PHYSICAL_SIZE)

            # Normalize points to [0,1] range within physical volume
            norm = (valid_pts - min_bound) / (max_bound - min_bound)
            # Convert to grid indices [0, GRID_SIZE-1]
            coords = (norm * (np.array(GRID_SIZE) - 1)).clip(0, np.array(GRID_SIZE)-1)
            idx = coords.astype(int)
            # Create 3D volume with improved intensity mapping
            volume_frame = np.zeros(GRID_SIZE, dtype=np.uint8)
            # min_z, max_z = valid_pts[:, 2].min(), valid_pts[:, 2].max()
            # Enhanced voxel coloring using height + distance combination
            for i in idx:
                x, y, z = i  # Grid coordinates
                if (0 <= x < GRID_SIZE[0] and 
                    0 <= y < GRID_SIZE[1] and 
                    0 <= z < GRID_SIZE[2]):
                    volume_frame[x, y, z] = max(volume_frame[x, GRID_SIZE[1] - y - 1, z], COLOR)
            # Improved post-processing
            volume_frame = gaussian_filter(volume_frame.astype(float), sigma=0.8)  # Smoother results
            volume_frame = np.clip(volume_frame, 0, 255).astype(np.uint8)

            # Append to the volume
            volume = np.maximum(volume, volume_frame)
            # --- Send to 3D Slicer ---
            # Create volume message with proper origin
            msg = pyigtl.ImageMessage(volume, device_name=device_name)
            # msg = pyigtl.PointMessage(valid_pts, device_name=device_name)
            msg.spacing = spacing
            msg.origin = [min_bound[0]*1000, min_bound[1]*1000, min_bound[2]*1000]  # mm
            server.send_message(msg)
            # Performance monitoring
            frame_count += 1
            if frame_count % 10 == 0:
                fps = 10 / (time.time() - prev_time)
                print(f"📦 Volume sent | Points: {valid_point_count} | FPS: {fps:.1f}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("⛔ Manually stopped.")
    finally:
        pipeline.stop()
        if MOUNTED:
            client.finalize()
        print("🧹 Camera stopped.")
if __name__ == "__main__":
    stream_depth_volume()