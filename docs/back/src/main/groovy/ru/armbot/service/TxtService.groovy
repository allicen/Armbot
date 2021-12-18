package ru.armbot.service

import io.micronaut.http.HttpStatus
import io.micronaut.http.exceptions.HttpStatusException
import io.micronaut.http.server.types.files.SystemFile
import jakarta.inject.Inject
import jakarta.inject.Singleton
import ru.armbot.domain.Coordinate
import ru.armbot.domain.LaunchFileRow

@Singleton
class TxtService {
    @Inject LogService logService

    SystemFile coordinateTxtFile (List<Coordinate> coordinateList) {

        try {
            File file = File.createTempFile('command_description', '.txt')

            coordinateList.each {
                file << "${it.name}:${it.x/1000} ${it.y/1000}" // перевод м в мм
                if (it.z != 0) {
                    file << " ${it.z/1000}"
                }
                file << "\n"
            }

            return new SystemFile(file).attach("command_description.txt")
        } catch (e) {
            logService.writeLog(this, "TXT export error: $e".toString(), 'error')
        }
        throw new HttpStatusException(HttpStatus.SERVICE_UNAVAILABLE, "error generating txt file")
    }

    SystemFile fileRowTxtFile (List<LaunchFileRow> fileRows) {

        fileRows.sort{it.sortOrder}

        try {
            File file = File.createTempFile('commands', '.txt')

            fileRows.each {
                file << "${it.coordinate.name} ${it.delay}\n"
            }

            return new SystemFile(file).attach("commands.txt")
        } catch (e) {
            logService.writeLog(this, "TXT export error: $e".toString(), 'error')
        }
        throw new HttpStatusException(HttpStatus.SERVICE_UNAVAILABLE, "error generating txt file")
    }
}
