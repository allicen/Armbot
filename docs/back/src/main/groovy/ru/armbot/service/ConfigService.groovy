package ru.armbot.service

import groovy.transform.CompileStatic
import io.micronaut.context.event.ApplicationEventListener
import io.micronaut.runtime.event.ApplicationStartupEvent
import jakarta.inject.Inject
import jakarta.inject.Singleton
import ru.armbot.domain.Config
import ru.armbot.domain.LogStatus
import ru.armbot.repository.ConfigRepository

@Singleton
@CompileStatic
class ConfigService implements ApplicationEventListener<ApplicationStartupEvent> {
    @Inject LogService logService
    @Inject ConfigRepository configRepository

    void loadConfig() {

        configRepository.deleteAll()

        String homePath = System.getProperty('user.home')
        String armbotConfigFile = "${homePath}/armbot-info/Armbot.txt"
        File configFile = new File(armbotConfigFile)

        if (configFile.exists()) {
            configFile.eachLine {line ->
                String[] lineArr = line.split('=')
                if (line.length() > 0 && line[0] != '#' && line.indexOf('=') > -1 && lineArr.length == 2) {
                    String key = lineArr[0].trim()
                    String value = lineArr[1].trim()

                    try {
                        configRepository.save(new Config(key: key, value: value))
                        logService.writeLog(this, "Конфиг ${line} успешно сохранен".toString(), LogStatus.INFO)
                    } catch (e) {
                        println("not save")
                        logService.writeLog(this, "Конфиг ${line} не сохранен, ошибка: $e".toString(), LogStatus.ERROR)
                    }
                }
            }

            logService.writeLog(this, "Файл с конфигами найден и обработан".toString(), LogStatus.INFO)
        } else {
            logService.writeLog(this, "Файла с конфигами ${armbotConfigFile} не существует".toString(), LogStatus.ERROR)
        }
    }

    @Override
    void onApplicationEvent(ApplicationStartupEvent event) {
        loadConfig()
    }
}
