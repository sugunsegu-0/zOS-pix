#pragma once
#ifndef PERCEPTION_GLOBALS_HH
#define PERCEPTION_GLOBALS_HH

#include <bits/stdc++.h>

// Number of Cameras
const uint32_t CAMERA_COUNT = 5;
uint32_t CAMERA_PORTS[5] = {0, 2, 4, 6, 8};
// const char* WINDOW_NAMES[CAMERA_COUNT] = {"front-center", "front-right", "front-left"};//, "back-right", "back-left"};
const char* WINDOW_NAMES[5] = {"front-center", "front-right", "front-left", "back-right", "back-left"};
const char* VIDEOS[5] = {
    // "/home/minuszero/driveworks-data/data/apps/cgf/onofframp/video_A0_FC_120.h264",
    // "/home/minuszero/driveworks-data/data/apps/cgf/onofframp/video_A2_RR_120.h264",
    // "/home/minuszero/driveworks-data/data/apps/cgf/onofframp/video_A1_RL_120.h264",
    // "/home/minuszero/driveworks-data/data/apps/cgf/onofframp/video_A2_XR_120.h264",
    // "/home/minuszero/driveworks-data/data/apps/cgf/onofframp/video_A1_XL_120.h264",

    // "/home/minuszero/Downloads/delhi4k-touse.mp4",
    // "/home/minuszero/Downloads/delhi4k-touse.mp4",
    // "/home/minuszero/Downloads/delhi4k-touse.mp4",
    // "/home/minuszero/Downloads/delhi4k-touse.mp4",
    // "/home/minuszero/Downloads/delhi4k-touse.mp4"
    // "/home/minuszero/Test-Videos/ToUse/test-touse.mp4",
    // "/home/minuszero/Test-Videos/ToUse/test-touse.mp4",
    // "/home/minuszero/Test-Videos/ToUse/test-touse.mp4",
    // "/home/minuszero/Test-Videos/ToUse/test-touse.mp4",
    // "/home/minuszero/Test-Videos/ToUse/test-touse.mp4",
    // "/home/minuszero/Test-/home/minuszero/work/zOS/src/SAL/dummy_drivers/delhi4k-touse.mp4ideos/ToUse/delhi4k-touse.mp4",
    // "/home/minuszero/Test-Videos/ToUse/delhi4k-touse.mp4",
    // "/home/minuszero/Test-Videos/ToUse/delhi4k-touse.mp4",
    // "/home/minuszero/Test-Videos/ToUse/delhi4k-touse.mp4",
    // "/home/minuszero/work/zOS/src/SAL/dummy_drivers/delhi4k-touse.mp4",    
    // "/home/minuszero/work/zOS/src/SAL/dummy_drivers/delhi4k-touse.mp4",
    // "/home/minuszero/work/zOS/src/SAL/dummy_drivers/delhi4k-touse.mp4",
    // "/home/minuszero/work/zOS/src/SAL/dummy_drivers/delhi4k-touse.mp4",
    // "/home/minuszero/work/zOS/src/SAL/dummy_drivers/delhi4k-touse.mp4"
    "/home/minuszero/Downloads/Seoul 4K - Driving Downtown - Skyline Shine.mp4",
    "/home/minuszero/Downloads/Seoul 4K - Driving Downtown - Skyline Shine.mp4",
    "/home/minuszero/Downloads/Seoul 4K - Driving Downtown - Skyline Shine.mp4",
    "/home/minuszero/Downloads/Seoul 4K - Driving Downtown - Skyline Shine.mp4",
    "/home/minuszero/Downloads/Seoul 4K - Driving Downtown - Skyline Shine.mp4"
    // "/home/minuszero/Downloads/SendAnywhere_757522/_out/out.mp4",
    // "/home/minuszero/Downloads/SendAnywhere_757522/_out/out.mp4",
    // "/home/minuszero/Downloads/SendAnywhere_757522/_out/out.mp4",
    // "/home/minuszero/Downloads/SendAnywhere_757522/_out/out.mp4",
    // "/home/minuszero/Downloads/SendAnywhere_757522/_out/out.mp4",
    // "/home/minuszero/Downloads/out.mp4",
    // "/home/minuszero/Downloads/out.mp4",
    // "/home/minuszero/Downloads/out.mp4",
    // "/home/minuszero/Downloads/out.mp4",
    // "/home/minuszero/Downloads/out.mp4",
    // "/home/minuszero/Downloads/VID_20221123_171215_1.mp4",
    // "/home/minuszero/Downloads/VID_20221123_171215_1.mp4",
    // "/home/minuszero/Downloads/VID_20221123_171215_1.mp4",
    // "/home/minuszero/Downloads/VID_20221123_171215_1.mp4",
    // "/home/minuszero/Downloads/VID_20221123_171215_1.mp4",
    // "/home/minuszero/Downloads/Rural India tour _ Indian village life _ Bihar village tour.mp4",
    // "/home/minuszero/Downloads/Rural India tour _ Indian village life _ Bihar village tour.mp4",
    // "/home/minuszero/Downloads/Rural India tour _ Indian village life _ Bihar village tour.mp4",
    // "/home/minuszero/Downloads/Rural India tour _ Indian village life _ Bihar village tour.mp4",
    // "/home/minuszero/Downloads/Rural India tour _ Indian village life _ Bihar village tour.mp4",
    // "/home/ade/output.mp4",
    // "/home/ade/output.mp4",
    // "/home/ade/output.mp4",
    // "/home/ade/output.mp4",
    // "/home/ade/output.mp4",
    // "/home/minuszero/Downloads/second_round_usb_cam_image_raw.mp4",
    // "/home/minuszero/Downloads/second_round_usb_cam_image_raw.mp4",
    // "/home/minuszero/Downloads/second_round_usb_cam_image_raw.mp4",
    // "/home/minuszero/Downloads/second_round_usb_cam_image_raw.mp4",
    // "/home/minuszero/Downloads/second_round_usb_cam_image_raw.mp4",
    // "/home/minuszero/Downloads/SendAnywhere_035345/rosbag_images/*.jpg",
    // "/home/minuszero/Downloads/SendAnywhere_035345/rosbag_images/*.jpg",
    // "/home/minuszero/Downloads/SendAnywhere_035345/rosbag_images/*.jpg",
    // "/home/minuszero/Downloads/SendAnywhere_035345/rosbag_images/*.jpg",
    // "/home/minuszero/Downloads/SendAnywhere_035345/rosbag_images/*.jpg"
    // "/home/prakhar/Downloads/Driving into Stunning Grindelwald - Switzerland 4K HDR.mp4",
    // "/home/prakhar/Downloads/Driving into Stunning Grindelwald - Switzerland 4K HDR.mp4",
    // "/home/prakhar/Downloads/Driving into Stunning Grindelwald - Switzerland 4K HDR.mp4",
    // "/home/prakhar/Downloads/Driving into Stunning Grindelwald - Switzerland 4K HDR.mp4",
    // "/home/prakhar/Downloads/Driving into Stunning Grindelwald - Switzerland 4K HDR.mp4",

};

// Width and Height of model input
uint32_t FRAME_WIDTH = 1280;
uint32_t FRAME_HEIGHT = 720;
// 

#endif