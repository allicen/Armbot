#pragma once
#ifndef logClass
#define logClass

#include <stdlib.h>
#include <stdio.h>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

class LogClass {
    public:
        void writeLog(const char*, char*);
        void logSimple(const char*, const char*, char*);
        void logGetCoordinates(const std::string*, const std::string*, const std::string*, char*);
        void logPrintPose(const geometry_msgs::Pose, char*);
        void logPrintJoints(const std::vector<double>, char*);
        void writeVersionLog(char*);
};

// Запись логов через shell
inline void LogClass::writeLog(const char* log, char* fileName) {
    char type[10] = "INFO";
    char shell[500] = "$ARMBOT_PATH/scripts/extended/functions/functions_log_cpp.sh '[";
    strcat (shell, type);
    strcat (shell, "] — ");
    strcat (shell, fileName);
    strcat (shell, " — ");
    strcat (shell, log);
    strcat (shell, "'");
    system (shell);
}

// Запись версии ПО через shell
inline void LogClass::writeVersionLog(char* fileName) {
    char type[10] = "INFO";
    char shell[500] = "$ARMBOT_PATH/scripts/extended/functions/functions_log_cpp_version.sh '[";
    strcat (shell, type);
    strcat (shell, "] — ");
    strcat (shell, fileName);
    strcat (shell, " — ");
    strcat (shell, "'");
    system (shell);
}

// Формирование простого лога (текст + данные)
inline void LogClass::logSimple(const char* logText, const char* data, char* filename) {
    char log[400];
    strcpy(log, logText);
    strcat(log, data);
    writeLog(log, filename);
}

// Запись в логи полученных координат
inline void LogClass::logGetCoordinates(const std::string* x, const std::string* y, const std::string* z, char* filename) {
    char log[255];
    strcpy(log, "Coordinates obtained for saving: x=");
    strcat(log, x->c_str());
    strcat(log, ", y=");
    strcat(log, y->c_str());
    strcat(log, ", z=");
    strcat(log, z->c_str());
    writeLog(log, filename);
}

// Запись в логи полученной позы
inline void LogClass::logPrintPose(const geometry_msgs::Pose pose, char* filename) {
    char log[255];
    strcpy(log, "Get pose: position_X=");
    strcat(log, boost::lexical_cast<std::string>(pose.position.x).c_str());
    strcat(log, ", position_Y=");
    strcat(log, boost::lexical_cast<std::string>(pose.position.y).c_str());
    strcat(log, ", position_Z=");
    strcat(log, boost::lexical_cast<std::string>(pose.position.z).c_str());
    strcat(log, ", orientation_X=");
    strcat(log, boost::lexical_cast<std::string>(pose.orientation.x).c_str());
    strcat(log, ", orientation_Y=");
    strcat(log, boost::lexical_cast<std::string>(pose.orientation.y).c_str());
    strcat(log, ", orientation_Z=");
    strcat(log, boost::lexical_cast<std::string>(pose.orientation.z).c_str());
    writeLog(log, filename);
}

// Запись в логи значений joints
inline void LogClass::logPrintJoints(const std::vector<double> joints, char* filename) {
    char log[255];
    strcpy(log, "Get joints value: 1=");
    strcat(log, boost::lexical_cast<std::string>(joints.at(0)).c_str());
    strcat(log, ", 2=");
    strcat(log, boost::lexical_cast<std::string>(joints.at(1)).c_str());
    strcat(log, ", 3=");
    strcat(log, boost::lexical_cast<std::string>(joints.at(2)).c_str());
    strcat(log, ", 4=");
    strcat(log, boost::lexical_cast<std::string>(joints.at(3)).c_str());
    writeLog(log, filename);
}

#endif
