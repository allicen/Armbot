package ru.armbot.service

import io.micronaut.http.HttpStatus
import io.micronaut.http.exceptions.HttpStatusException
import io.micronaut.http.server.types.files.SystemFile
import jakarta.inject.Singleton

@Singleton
class CoordinateTxtService {
    SystemFile txtFile (List coordinateList) {

        try {
            File file = File.createTempFile('test', '.txt')

            coordinateList.each {
                file << "${it.name}:${it.x} ${it.y} ${it.z}"
            }

            return new SystemFile(file).attach("test.txt")
        } catch (e) {
            println("EXCEL ERROR: " + e)
        }
        throw new HttpStatusException(HttpStatus.SERVICE_UNAVAILABLE, "error generating excel file")
    }
}
