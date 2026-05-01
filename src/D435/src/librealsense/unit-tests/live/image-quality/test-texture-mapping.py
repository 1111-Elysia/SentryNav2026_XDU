# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

# test:device D400*
# test:donotrun
import pyrealsense2 as rs
from rspy import log, test
import numpy as np
import cv2
from iq_helper import find_roi_location, get_roi_from_frame, is_color_close, WIDTH, HEIGHT


NUM_FRAMES = 100 # Number of frames to check
COLOR_TOLERANCE = 60 # Acceptable per-channel deviation in RGB values
DEPTH_TOLERANCE = 0.05  # Acceptable deviation from expected depth in meters
FRAMES_PASS_THRESHOLD =0.8 # Percentage of frames that needs to pass
DEBUG_MODE = False

# expected colors (insertion order -> mapped row-major to 3x3 grid)
expected_colors = {
    "red":   (132, 60, 60),
    "green": (40, 84, 72),
    "blue":  (20, 67, 103),
    "black": (35, 35, 35),
    "white": (150, 150, 150),
    "gray": (90, 90, 90),
    "purple": (56, 72, 98),
    "orange": (136, 86, 70),
    "yellow": (166, 142, 80),
}
EXPECTED_DEPTH = 0.53  # meters - all of the target is at this distance for this test
R = 20  # radius area around the center to sample depth from (in pixels)
# list of color names in insertion order -> used left->right, top->bottom
color_names = list(expected_colors.keys())

# since this is a 3x3 grid, we have separators at 0, 1/3, 2/3, 1 of width and height
# to calculate the centers, we take the middle of each cell - between the separators
# instead of taking exact center, we offset a bit to the general center to avoid sampling out of the area
xs = [1.5 * WIDTH / 6.0, WIDTH / 2.0, 4.5 * WIDTH / 6.0]
ys = [1.5 * HEIGHT / 6.0, HEIGHT / 2.0, 4.5 * HEIGHT / 6.0]
centers = [(x, y) for y in ys for x in xs]


def draw_debug(a4_page_bgr):
    """
    Simple debug view:
      - left: camera frame
      - right: focused view on the A4 page with grid and color names
    """
    vertical_lines = [WIDTH / 3.0, 2.0 * WIDTH / 3.0]
    horizontal_lines = [HEIGHT / 3.0, 2.0 * HEIGHT / 3.0]
    H, W = a4_page_bgr.shape[:2]

    # draw grid on a4 page image
    for x in vertical_lines:
        cv2.line(a4_page_bgr, (int(x), 0), (int(x), H - 1), (255, 255, 255), 2)
    for y in horizontal_lines:
        cv2.line(a4_page_bgr, (0, int(y)), (W - 1, int(y)), (255, 255, 255), 2)

    # label centers with color names
    for i, (cx, cy) in enumerate(centers):
        cx_i, cy_i = int(round(cx)), int(round(cy))
        lbl = color_names[i] if i < len(color_names) else str(i)
        # we sample from an area - mark it
        [cv2.circle(a4_page_bgr, (cx_i + dx, cy_i + dy), 1, (255, 255, 255), -1)
         for dy in range(-R, R) for dx in range(-R, R)]
        # white marker with black text for readability
        cv2.putText(a4_page_bgr, lbl, (cx_i + 12, cy_i + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)

    # resize and display side by side
    height = 600
    a4_page_width = int(a4_page_bgr.shape[1] * (height / a4_page_bgr.shape[0]))
    right = cv2.resize(a4_page_bgr, (a4_page_width, height))
    return right


dev, ctx = test.find_first_device_or_exit()

def run_test(depth_resolution, depth_fps, color_resolution, color_fps):
    try:
        pipeline = rs.pipeline(ctx)
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, depth_resolution[0], depth_resolution[1], rs.format.z16, depth_fps)
        cfg.enable_stream(rs.stream.color, color_resolution[0], color_resolution[1], rs.format.bgr8, color_fps)
        if not cfg.can_resolve(pipeline):
            log.f(f"Basic config not supported! Depth: {depth_resolution[0]}x{depth_resolution[1]}@{depth_fps}fps, "
                    f"Color: {color_resolution[0]}x{color_resolution[1]}@{color_fps}fps")
            return
        pipeline_profile = pipeline.start(cfg)
        depth_sensor = pipeline_profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        colorizer = rs.colorizer()
        for i in range(60):  # skip initial frames
            pipeline.wait_for_frames()

        align = rs.align(rs.stream.color)
        color_passes = {name: 0 for name in color_names}
        depth_passes = {name: 0 for name in color_names}

        # find region of interest (page) and get the transformation matrix
        find_roi_location(pipeline, (4, 5, 6, 7), DEBUG_MODE)  # markers in the lab are 4,5,6,7

        for i in range(NUM_FRAMES):
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            color_frame_roi = get_roi_from_frame(color_frame)
            depth_frame_roi = get_roi_from_frame(depth_frame)

            # check colors
            for idx, (x, y) in enumerate(centers):
                color = color_names[idx] if idx < len(color_names) else str(idx)
                expected_rgb = expected_colors[color]
                x = int(round(x))
                y = int(round(y))

                # check color OK
                b, g, r = (int(v) for v in color_frame_roi[y, x])  # stream is BGR, convert to RGB
                pixel = (r, g, b)
                if is_color_close(pixel, expected_rgb, COLOR_TOLERANCE):
                    color_passes[color] += 1
                else:
                    log.d(f"Frame {i} - {color} at ({x},{y}) sampled: {pixel} too far from expected {expected_rgb}")

                # because we align depth to color, we get some noise at some areas - get average of valid values nearby
                sample_area = depth_frame_roi[y - R:y + R, x - R:x + R]
                invalid_values = sample_area[sample_area < 300]  # most cameras have min depth ~300mm
                valid_values = sample_area[sample_area >= 300]
                # check if there are a lot of invalid values - if it happens in many frames, the test will fail
                if invalid_values.size > sample_area.size * 0.4:
                    log.d(f"Frame {i} - {color} at ({x},{y}): too many invalid depth values, skipping "
                          f"({invalid_values.size} invalid vs {sample_area.size} total)")
                    continue
                raw_depth = valid_values.mean()
                depth_value = raw_depth * depth_scale  # Convert to meters

                if abs(depth_value - EXPECTED_DEPTH) <= DEPTH_TOLERANCE:
                    depth_passes[color] += 1
                else:
                    log.d(f"Frame {i} - {color} at ({x},{y}): {depth_value:.3f}m ≠ {EXPECTED_DEPTH:.3f}m")

                if DEBUG_MODE:
                    # To see the depth on top of the color, blend the images
                    depth_image = get_roi_from_frame(colorizer.colorize(depth_frame))
                    color_image = color_frame_roi

                    alpha = 0.8  # transparency factor
                    overlay = cv2.addWeighted(depth_image, 1 - alpha, color_image, alpha, 0)

                    # crop the image according to the markers found
                    dbg = draw_debug(overlay)
                    cv2.imshow('Overlay', dbg)
                    cv2.waitKey(1)

        # Check per-color pass threshold
        min_passes = int(NUM_FRAMES * FRAMES_PASS_THRESHOLD)

        log.i("\n--- Color Results ---")
        for name, count in color_passes.items():
            log.i(f"{name.title()} passed in {count}/{NUM_FRAMES} frames")
            test.check(count >= min_passes, f"{name.title()} color failed in too many frames")

        log.i("\n--- Depth Results ---")
        for name, count in depth_passes.items():
            log.i(f"{name.title()} depth passed in {count}/{NUM_FRAMES} frames")
            test.check(count >= min_passes, f"{name.title()} depth failed in too many frames")

    except Exception as e:
        test.unexpected_exception()
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()
        test.finish()

log.d("context:", test.context)
if "nightly" not in test.context:
    depth_configs = [((1280, 720), 30)]
else:
    depth_configs = [
        ((640,480), 30),
        ((1280,720), 30),
    ]

color_configs = depth_configs

for depth_cfg in depth_configs:
    for color_cfg in color_configs:
        test.start("Texture Mapping Test",
                   f"Color: {color_cfg[0][0]}x{color_cfg[0][1]}@{color_cfg[1]}fps | "
                   f"Depth: {depth_cfg[0][0]}x{depth_cfg[0][1]}@{depth_cfg[1]}fps")
        run_test(*depth_cfg, *color_cfg)
        test.finish()


test.print_results_and_exit()
