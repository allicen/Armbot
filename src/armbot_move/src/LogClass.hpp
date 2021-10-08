#pragma once
#ifndef logClass
#define logClass

#include <stdlib.h>
#include <stdio.h>

class LogClass {
    public:
        void writeLog(const char*, char*);
};

inline void LogClass::writeLog(const char* log, char* fileName) {

    char shell[255] = "$ARMBOT_PATH/scripts/functions_log_cpp.sh '";
    strcat (shell, fileName);
    strcat (shell, " --- ");
    strcat (shell, log);
    strcat (shell, "'");
    system (shell);
}

#endif
