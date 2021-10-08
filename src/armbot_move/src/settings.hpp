#pragma once
#ifndef settings
#define settings

#define commandDescriptionFile std::string(getenv("ARMBOT_PATH")) + "/data/command_description.txt"

float zPositionDefault = 0.036107;
float zPositionDefaultDown = 0.0198367;

typedef struct GripperOrientationDefault {
    float x = 0.00724986;
    float y = 0.999963;
    float z = 3.30262e-05;
    float w = 0.00455517;
} Orientation;

#endif