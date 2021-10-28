package ru.armbot.service

import io.micronaut.http.HttpStatus
import io.micronaut.http.exceptions.HttpStatusException
import io.micronaut.http.server.types.files.SystemFile
import jakarta.inject.Singleton

@Singleton
class CoordinateTxtService {
    SystemFile txtFile (List coordinateList) {

        try {
            File file = File.createTempFile('points', '.txt')

            coordinateList.each {
                file << "${it.name}:${it.x} ${it.y} ${it.z}\n"
            }

            return new SystemFile(file).attach("points.txt")
        } catch (e) {
            println("TXT ERROR: " + e)
        }
        throw new HttpStatusException(HttpStatus.SERVICE_UNAVAILABLE, "error generating txt file")
    }
}
