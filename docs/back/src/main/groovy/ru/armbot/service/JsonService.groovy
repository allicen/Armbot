package ru.armbot.service

import groovy.json.JsonBuilder
import io.micronaut.http.HttpStatus
import io.micronaut.http.exceptions.HttpStatusException
import io.micronaut.http.server.types.files.SystemFile
import jakarta.inject.Singleton
import ru.armbot.dto.SessionStateDto

@Singleton
class JsonService {
    SystemFile sessionJsonFile (SessionStateDto sessionStateDto) {

        try {
            File file = File.createTempFile('session', '.json')

            def coordinateList = []
            def launchFileRows = []
            sessionStateDto.coordinateList.each {
                coordinateList += new JsonBuilder(it)
            }

            sessionStateDto.launchFileRowList.each {
                launchFileRows += new JsonBuilder(it)
            }

            file << new JsonBuilder(sessionStateDto)
            return new SystemFile(file).attach("session.json")
        } catch (e) {
            println("TXT ERROR: " + e)
        }
        throw new HttpStatusException(HttpStatus.SERVICE_UNAVAILABLE, "error generating txt file")
    }
}
