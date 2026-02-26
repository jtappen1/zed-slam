#!/usr/bin/env python3


import pyzed.sl as sl
import csv
import math

MIN_DIST = 0.1  # meters between saved waypoints — tune this

def get_yaw(pose):
    rv = pose.get_rotation_vector()
    return math.degrees(rv[1])  # Y component = yaw in RIGHT_HANDED_Y_UP

def teach():
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.camera_fps = 30
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT
    zed.open(init_params)

    tracking_params = sl.PositionalTrackingParameters()
    tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
    tracking_params.enable_area_memory = True
    # tracking_params.set_floor_as_origin = True
    tracking_params.enable_imu_fusion = True
    tracking_params.set_gravity_as_origin = True 
    tracking_params.enable_2d_ground_mode = True

    zed.enable_positional_tracking(tracking_params)

    tracking_status = zed.get_positional_tracking_status()
    runtime_params = sl.RuntimeParameters()

    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            tracking_state = zed.get_position(sl.Pose())
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                print("[INFO] Positional tracking ready.")
                break

    pose = sl.Pose()
    path = []
    last_x, last_y = None, None

    print("[TEACH] Recording path... Press Ctrl+C to stop.")
    try:
        while True:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                state = zed.get_position(pose)
                mem_status = tracking_status.spatial_memory_status
                if mem_status == sl.SPATIAL_MEMORY_STATUS.LOOP_CLOSED:
                    print("[] Loop closure detected!")
                elif mem_status == sl.SPATIAL_MEMORY_STATUS.KNOWN_MAP:
                    print("[] KNOWN MAP")
                elif mem_status == sl.SPATIAL_MEMORY_STATUS.LOST:
                    print("[] LOST")

                if state == sl.POSITIONAL_TRACKING_STATE.OK:
                    t = pose.get_translation(sl.Translation())
                    x = t.get()[0]
                    y = t.get()[1]
        
                    if last_x is None or math.hypot(x - last_x, y - last_y) >= MIN_DIST:
                        yaw = get_yaw(pose)
                        # Rotate 90: world -y becomes +x, this is due to the pos tracking saving y as forward.
                        rotated_x = y
                        rotated_y = -x
                        path.append({"x": rotated_x, "y": rotated_y})
                        last_x, last_y = x, y

    except KeyboardInterrupt:
        pass

    file_path = "/home/nvidia/zed_ws/src/zedx_pure_pursuit/path/"

    with open(file_path + "waypoints.csv", "w", newline="") as f:
        fieldnames = path[0].keys()
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(path)

    print(f"[TEACH] Saved {len(path)} poses to way_points.csv")

    zed.save_area_map(file_path + "my_area.area")
    print("[TEACH] Saved area map.")

    zed.disable_positional_tracking()
    zed.close()

if __name__ == "__main__":
    teach()
