#pragma once

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>

#include "LogClass.hpp"
char FILE_SETTINGS[30] = "settings.hpp";
LogClass logConfig;

#ifndef settings
#define settings

#define commandDescriptionFile std::string(getenv("ARMBOT_PATH")) + "/data/command_description.txt"
#define websocketUrl "ws://localhost:9080/ws/coordinate"

float zPositionDefault = 0.05;
float zPositionDefaultDown = 0.012;
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

bool saveWebSocket = true;
bool saveToFile = false;

class SettingsClass {
    public:
        void update();
};


// Перезаписать конфиги из файла
inline void SettingsClass::update() {
    Orientation orientation;
    DefaultPosition position;

    std::ifstream file("/home/armbot-info/Armbot.txt");
    std::string line;

    while (std::getline(file, line)) {
        std::string delimiter = "=";

        if (line.length() > 0 && line.at(0) != '#' && std::count(line.cbegin(), line.cend(), '=') == 1) {
            size_t pos = 0;
            std::string token;
            while ((pos = line.find(delimiter)) != std::string::npos) {
                token = line.substr(0, pos);
                line.erase(0, pos + delimiter.length());
            }

            boost::trim_left(token);
            boost::trim_right(token);
            boost::trim_left(line);
            boost::trim_right(line);

            if (strcmp("zPositionDefault", token.c_str()) == 0) {
                zPositionDefault = stof(line);
                logConfig.logSimple("Переопределена переменная zPositionDefault, новое значение: ",  boost::lexical_cast<std::string>(zPositionDefault).c_str(), FILE_SETTINGS);

            } else if (strcmp("zPositionDefaultDown", token.c_str()) == 0) {
                zPositionDefaultDown = stof(line);
                logConfig.logSimple("Переопределена переменная zPositionDefaultDown, новое значение: ",  boost::lexical_cast<std::string>(zPositionDefaultDown).c_str(), FILE_SETTINGS);

            } else if (strcmp("Orientation.x", token.c_str()) == 0) {
                orientation.x = stof(line);
                logConfig.logSimple("Переопределена переменная Orientation.x, новое значение: ",  boost::lexical_cast<std::string>(orientation.x).c_str(), FILE_SETTINGS);

            } else if (strcmp("Orientation.y", token.c_str()) == 0) {
                orientation.y = stof(line);
                logConfig.logSimple("Переопределена переменная Orientation.y, новое значение: ",  boost::lexical_cast<std::string>(orientation.y).c_str(), FILE_SETTINGS);

            } else if (strcmp("Orientation.z", token.c_str()) == 0) {
                orientation.z = stof(line);
                logConfig.logSimple("Переопределена переменная Orientation.z, новое значение: ",  boost::lexical_cast<std::string>(orientation.z).c_str(), FILE_SETTINGS);

            } else if (strcmp("Orientation.w", token.c_str()) == 0) {
                orientation.w = stof(line);
                logConfig.logSimple("Переопределена переменная Orientation.w, новое значение: ",  boost::lexical_cast<std::string>(orientation.w).c_str(), FILE_SETTINGS);

            } else if (strcmp("DefaultPosition.x", token.c_str()) == 0) {
                position.x = stof(line);
                logConfig.logSimple("Переопределена переменная DefaultPosition.x, новое значение: ",  boost::lexical_cast<std::string>(position.x).c_str(), FILE_SETTINGS);

            } else if (strcmp("DefaultPosition.y", token.c_str()) == 0) {
                position.y = stof(line);
                logConfig.logSimple("Переопределена переменная DefaultPosition.y, новое значение: ",  boost::lexical_cast<std::string>(position.y).c_str(), FILE_SETTINGS);

            } else if (strcmp("DefaultPosition.z", token.c_str()) == 0) {
                position.z = stof(line);
                logConfig.logSimple("Переопределена переменная DefaultPosition.z, новое значение: ",  boost::lexical_cast<std::string>(position.z).c_str(), FILE_SETTINGS);

            } else if (strcmp("saveWebSocket", token.c_str()) == 0) {
                saveWebSocket = (strcmp("true", line.c_str()) == 0);
                logConfig.logSimple("Переопределена переменная saveWebSocket, новое значение: ",  boost::lexical_cast<std::string>(saveWebSocket).c_str(), FILE_SETTINGS);

            } else if (strcmp("saveToFile", token.c_str()) == 0) {
                saveToFile = (strcmp("true", line.c_str()) == 0);
                logConfig.logSimple("Переопределена переменная saveToFile, новое значение: ",  boost::lexical_cast<std::string>(saveToFile).c_str(), FILE_SETTINGS);
            }
        }
    }
}

#endif
