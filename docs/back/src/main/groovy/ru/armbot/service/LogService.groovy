package ru.armbot.service

import jakarta.inject.Singleton
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import ru.armbot.domain.LogStatus

@Singleton
class LogService {
    void writeLog(def classLog, String logText, LogStatus logStatus = LogStatus.INFO) {

        String className = classLog.getClass().simpleName
        Logger logger = LoggerFactory.getLogger(className)

        if (logStatus == LogStatus.INFO) {
            logger.info(logText)
        } else {
            logger.error(logText)
        }
    }
}
