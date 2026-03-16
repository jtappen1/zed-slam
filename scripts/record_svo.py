#!/usr/bin/env python3

import pyzed.sl as sl
import sys
import time

def main():
    # Create ZED camera object
    zed = sl.Camera()

    # Initialize camera and camera parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.SVGA
    init_params.camera_fps = 60
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT


    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Error opening ZED camera")
        exit(1)

    runtime_parameters = sl.RuntimeParameters()

    # NOTE:  It looks like this is not needed for recording SVOs, will keep this in just in case it is helpful in the future
    # # Enable positional tracking and parameters
    # tracking_parameters = sl.PositionalTrackingParameters()
    # tracking_parameters.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
    # tracking_parameters.enable_area_memory = True 
    # tracking_parameters.enable_pose_smoothing = False 
    # tracking_parameters.set_floor_as_origin = True 
    # tracking_parameters.enable_imu_fusion = True 
    # tracking_parameters.set_gravity_as_origin = True 
    # tracking_parameters.enable_2d_ground_mode = False
    # zed.enable_positional_tracking(tracking_parameters)

    # print("[INFO] Waiting for positional tracking to initialize...")
    # while True:
    #     if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
    #         tracking_state = zed.get_position(sl.Pose())
    #         if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
    #             print("[INFO] Positional tracking ready.")
    #             break


    # Enable recording to SVO
    output_path = sys.argv[1]
    recordingParameters = sl.RecordingParameters()
    recordingParameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264
    recordingParameters.video_filename = output_path

    if zed.enable_recording(recordingParameters) != sl.ERROR_CODE.SUCCESS:
        print("Error enabling recording")
        zed.close()
        exit(1)

    print("[INFO] Recording... Press Ctrl+C to stop")
    try:
        frame_count = 0
        while True:
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                frame_count += 1
                if frame_count % 50 == 0:
                    print(f"Frame {frame_count}")
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[INFO] Stopping recording...")

    # Stop recording and close
    zed.disable_recording()
    zed.close()
    print(f"[INFO] Recording saved at {output_path}")

if __name__ == "__main__":
    main()
