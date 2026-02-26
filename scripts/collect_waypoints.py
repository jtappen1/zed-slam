import pyzed.sl as sl
import csv
import math

MIN_DIST = 0.5  # meters between saved waypoints — tune this

def get_yaw(pose):
    rv = pose.get_rotation_vector()
    return math.degrees(rv[1])  # Y component = yaw in RIGHT_HANDED_Y_UP

def teach():
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.SVGA
    init_params.camera_fps = 30
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    zed.open(init_params)

    tracking_params = sl.PositionalTrackingParameters()
    tracking_params.enable_area_memory = True
    tracking_params.set_floor_as_origin = False
    tracking_params.enable_imu_fusion = True
    zed.enable_positional_tracking(tracking_params)

    runtime_params = sl.RuntimeParameters()
    pose = sl.Pose()
    path = []
    last_x, last_y = None, None

    print("[TEACH] Recording path... Press Ctrl+C to stop.")
    try:
        while True:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                state = zed.get_position(pose)
                if state == sl.POSITIONAL_TRACKING_STATE.OK:
                    t = pose.get_translation(sl.Translation())
                    x = t.get()[0]
                    y = t.get()[2]

                    if last_x is None or math.hypot(x - last_x, y - last_y) >= MIN_DIST:
                        yaw = get_yaw(pose)
                        path.append({"x": x, "y": y})
                        last_x, last_y = x, y

    except KeyboardInterrupt:
        pass

    with open("way_points.csv", "w", newline="") as f:
        fieldnames = path[0].keys()
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(path)

    print(f"[TEACH] Saved {len(path)} poses to way_points.csv")

    zed.save_area_map("my_area.area")
    print("[TEACH] Saved area map.")

    zed.disable_positional_tracking()
    zed.close()

if __name__ == "__main__":
    teach()
