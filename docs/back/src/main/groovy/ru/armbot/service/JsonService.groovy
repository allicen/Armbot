package ru.armbot.service

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
            file << new JsonSlurper(sessionStateDto)
            return new SystemFile(file).attach("session.json")
        } catch (e) {
            println("TXT ERROR: " + e)
        }
        throw new HttpStatusException(HttpStatus.SERVICE_UNAVAILABLE, "error generating txt file")
    }
}
