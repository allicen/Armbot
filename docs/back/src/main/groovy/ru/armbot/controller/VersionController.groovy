package ru.armbot.controller

import io.micronaut.core.io.ResourceResolver
import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Get
import ru.armbot.domain.ResponseStatus
import ru.armbot.dto.ResponseDto

@Controller("/version")
class VersionController {

    VersionController() { }

    @Get(value = "/get")
    ResponseDto getVersion() {

        def env = System.getenv()
        def armbotPath = env['ARMBOT_PATH']

        // Файл с версией должен лежать в корне проекта Armbot
        // Если этого файла нет, то берем версию из ресурсов UI
        String armbotFilePath = "${armbotPath}/version"
        Map<String, String> data = [:]

        File versionFile = new File(armbotFilePath)
        if (versionFile.exists()) {
            versionFile.eachLine {
                data.put(addVersionLine(it).key, addVersionLine(it).value)
            }
        } else {
            new ResourceResolver().getResources("classpath:version").forEach(url->
                url.eachLine{
                    data.put(addVersionLine(it).key, addVersionLine(it).value)
                }
            )
        }

        return new ResponseDto(status: ResponseStatus.SUCCESS, message: "Данные версии получены", details: data)
    }

    Map<String, String> addVersionLine(String line) {
        String key = "NULL"
        String value = "NULL"
        String[] lineStr = line.split("=")
        if (lineStr.length > 0) {
            key = lineStr[0]
        }

        if (lineStr.length > 1) {
            value = lineStr[1]
        }

        return [key: key, value: value]
    }
}
