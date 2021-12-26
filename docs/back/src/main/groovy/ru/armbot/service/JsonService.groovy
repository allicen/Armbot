package ru.armbot.service

import groovy.json.JsonBuilder
import io.micronaut.http.HttpStatus
import io.micronaut.http.exceptions.HttpStatusException
import io.micronaut.http.server.types.files.SystemFile
import jakarta.inject.Inject
import jakarta.inject.Singleton
import ru.armbot.domain.LogStatus
import ru.armbot.dto.SessionStateDto

@Singleton
class JsonService {
    @Inject LogService logService

    SystemFile sessionJsonFile (SessionStateDto sessionStateDto) {

        try {
            File file = new File('armbot-session.json')

            def coordinateList = []
            def launchFileRows = []
            sessionStateDto.coordinateList.each {
                coordinateList += new JsonBuilder(it)
            }

            sessionStateDto.launchFileRowList.each {
                launchFileRows += new JsonBuilder(it)
            }

            file << new JsonBuilder(sessionStateDto)
            return new SystemFile(file).attach("armbot-session.json")
        } catch (e) {
            logService.writeLog(this, "JSON export error: $e".toString(), LogStatus.ERROR)
        }
        throw new HttpStatusException(HttpStatus.SERVICE_UNAVAILABLE, "error generating txt file")
    }
}
