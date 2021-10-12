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
    char type[10] = "INFO";
    char shell[255] = "$ARMBOT_PATH/scripts/extended/functions/functions_log_cpp.sh '[";
    strcat (shell, type);
    strcat (shell, "] — ");
    strcat (shell, fileName);
    strcat (shell, " — ");
    strcat (shell, log);
    strcat (shell, "'");
    system (shell);
}

#endif
