package ru.armbot.service

import io.micronaut.http.HttpStatus
import io.micronaut.http.exceptions.HttpStatusException
import io.micronaut.http.server.types.files.SystemFile
import jakarta.inject.Singleton
import ru.armbot.domain.Coordinate
import ru.armbot.domain.LaunchFileRow

@Singleton
class txtService {
    SystemFile coordinateTxtFile (List<Coordinate> coordinateList) {

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

    SystemFile fileRowTxtFile (List<LaunchFileRow> fileRows) {

        fileRows.sort{it.sortOrder}

        try {
            File file = File.createTempFile('launch-file', '.txt')

            fileRows.each {
                file << "${it.coordinate.name} ${it.delay}\n"
            }

            return new SystemFile(file).attach("launch-file.txt")
        } catch (e) {
            println("TXT ERROR: " + e)
        }
        throw new HttpStatusException(HttpStatus.SERVICE_UNAVAILABLE, "error generating txt file")
    }
}
