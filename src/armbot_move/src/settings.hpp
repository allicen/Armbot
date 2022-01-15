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

// Координата не указана
float coordinateNone = -1000;

float defaultOrientation_x = 0.999973;
float defaultOrientation_y = 0.00734455;
float defaultOrientation_z = -3.69362e-05;
float defaultOrientation_w = -2.76376e-06;

float defaultPosition_x = 0.15731;
float defaultPosition_y = 0.00169707;
float defaultPosition_z = 0.119355;

// Сохранение кординаты в БД
bool saveWebSocket = true;

// Сохранение координаты в файл data/command_description.txt
bool saveToFile = false;

// Скорость шаговых двигателей
// Рекомендуемый диапазон: от 500 до 3000
// Максимум можно установить 11000
int maxSpeedStepperMotor = 500;

class SettingsClass {
    public:
        void update();
};


// Перезаписать конфиги из файла
inline void SettingsClass::update() {
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

            } else if (strcmp("defaultOrientation_x", token.c_str()) == 0) {
                defaultOrientation_x = stof(line);
                logConfig.logSimple("Переопределена переменная defaultOrientation_x, новое значение: ",  boost::lexical_cast<std::string>(defaultOrientation_x).c_str(), FILE_SETTINGS);

            } else if (strcmp("defaultOrientation_y", token.c_str()) == 0) {
                defaultOrientation_y = stof(line);
                logConfig.logSimple("Переопределена переменная defaultOrientation_y, новое значение: ",  boost::lexical_cast<std::string>(defaultOrientation_y).c_str(), FILE_SETTINGS);

            } else if (strcmp("defaultOrientation_z", token.c_str()) == 0) {
                defaultOrientation_z = stof(line);
                logConfig.logSimple("Переопределена переменная defaultOrientation_z, новое значение: ",  boost::lexical_cast<std::string>(defaultOrientation_z).c_str(), FILE_SETTINGS);

            } else if (strcmp("defaultOrientation_w", token.c_str()) == 0) {
                defaultOrientation_w = stof(line);
                logConfig.logSimple("Переопределена переменная defaultOrientation_w, новое значение: ",  boost::lexical_cast<std::string>(defaultOrientation_w).c_str(), FILE_SETTINGS);

            } else if (strcmp("defaultPosition_x", token.c_str()) == 0) {
                defaultPosition_x = stof(line);
                logConfig.logSimple("Переопределена переменная defaultPosition_x, новое значение: ",  boost::lexical_cast<std::string>(defaultPosition_x).c_str(), FILE_SETTINGS);

            } else if (strcmp("defaultPosition_y", token.c_str()) == 0) {
                defaultPosition_y = stof(line);
                logConfig.logSimple("Переопределена переменная defaultPosition_y, новое значение: ",  boost::lexical_cast<std::string>(defaultPosition_y).c_str(), FILE_SETTINGS);

            } else if (strcmp("defaultPosition_z", token.c_str()) == 0) {
                defaultPosition_z = stof(line);
                logConfig.logSimple("Переопределена переменная defaultPosition_z, новое значение: ",  boost::lexical_cast<std::string>(defaultPosition_z).c_str(), FILE_SETTINGS);

            } else if (strcmp("saveWebSocket", token.c_str()) == 0) {
                saveWebSocket = (strcmp("true", line.c_str()) == 0);
                logConfig.logSimple("Переопределена переменная saveWebSocket, новое значение: ",  boost::lexical_cast<std::string>(saveWebSocket).c_str(), FILE_SETTINGS);

            } else if (strcmp("saveToFile", token.c_str()) == 0) {
                saveToFile = (strcmp("true", line.c_str()) == 0);
                logConfig.logSimple("Переопределена переменная saveToFile, новое значение: ",  boost::lexical_cast<std::string>(saveToFile).c_str(), FILE_SETTINGS);

            } else if (strcmp("maxSpeedStepperMotor", token.c_str()) == 0) {
                maxSpeedStepperMotor = atoi(line.c_str());
                logConfig.logSimple("Переопределена переменная maxSpeedStepperMotor, новое значение: ",  boost::lexical_cast<std::string>(maxSpeedStepperMotor).c_str(), FILE_SETTINGS);
            }
        }
    }
}

#endif
