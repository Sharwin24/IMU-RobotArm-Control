#pragma once

constexpr int STEPS_PER_REV = 6400;
constexpr int LINK_RPM = 200; // [RPM]
constexpr float LINK_STEPS_PER_SEC = STEPS_PER_REV * (LINK_RPM / 60.0f); // [steps/sec]

constexpr int LINK_1_LENGTH = 125; // [mm]
constexpr int LINK_1_STEP_PIN = 2;
constexpr int LINK_1_DIR_PIN = 3;

constexpr int LINK_2_LENGTH = 100; // [mm]
constexpr int LINK_2_STEP_PIN = 4;
constexpr int LINK_2_DIR_PIN = 5;

constexpr int LINK_3_LENGTH = 40; // [mm]
constexpr int LINK_3_STEP_PIN = 6;
constexpr int LINK_3_DIR_PIN = 7;
