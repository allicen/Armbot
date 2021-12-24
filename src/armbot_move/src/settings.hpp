#pragma once
#ifndef settings
#define settings

#define commandDescriptionFile std::string(getenv("ARMBOT_PATH")) + "/data/command_description.txt"

float zPositionDefault = 0.066107;
float zPositionDefaultDown = 0.0098367;
float zPositionNone = -1000;

typedef struct GripperOrientationDefault {
    float x = 0.999973;
    float y = 0.00734455;
    float z = -3.69362e-05;
    float w = -2.76376e-06;
} Orientation;

typedef struct DefaultPosition {
	float x = 0.15731;
	float y = 0.00169707;
	float z = 0.119355;
} DefaultPosition;

#define websocketUrl "ws://localhost:9080/ws/coordinate"

#define saveWebSocket false
#define saveToFile true

#endif
