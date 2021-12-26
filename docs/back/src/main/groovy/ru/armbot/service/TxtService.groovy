package ru.armbot.service

import io.micronaut.http.HttpStatus
import io.micronaut.http.exceptions.HttpStatusException
import io.micronaut.http.server.types.files.SystemFile
import jakarta.inject.Inject
import jakarta.inject.Singleton
import ru.armbot.domain.Coordinate
import ru.armbot.domain.LaunchFileRow
import ru.armbot.domain.LogStatus

@Singleton
class TxtService {
    @Inject LogService logService

    SystemFile coordinateTxtFile (List<Coordinate> coordinateList) {

        try {
            File file = getCoordinateTxtFile (coordinateList)
            return new SystemFile(file).attach("command_description.txt")
        } catch (e) {
            logService.writeLog(this, "TXT export error: $e".toString(), LogStatus.ERROR)
        }
        throw new HttpStatusException(HttpStatus.SERVICE_UNAVAILABLE, "error generating txt file")
    }

    SystemFile fileRowTxtFile (List<LaunchFileRow> fileRows) {

        fileRows.sort{it.sortOrder}

        try {
            File file = getFileRowTxtFile (fileRows)
            return new SystemFile(file).attach("commands.txt")
        } catch (e) {
            logService.writeLog(this, "TXT export error: $e".toString(), LogStatus.ERROR)
        }
        throw new HttpStatusException(HttpStatus.SERVICE_UNAVAILABLE, "error generating txt file")
    }

    File getCoordinateTxtFile (List<Coordinate> coordinateList, String fileName = 'command_description', boolean createByDefaultPath = true) {
        File file = null
        try {
            file = createByDefaultPath ? File.createTempFile(fileName, '.txt') : new File("${fileName}")

            coordinateList.each {
                file << "${it.name}:${it.x/1000} ${it.y/1000}" // перевод м в мм
                if (it.z != 0) {
                    file << " ${it.z/1000}"
                }
                file << "\n"
            }
        } catch (e) {
            logService.writeLog(this, "TXT generate error: $e".toString(), LogStatus.ERROR)
        }

        return file
    }

    File getFileRowTxtFile (List<LaunchFileRow> fileRows, String fileName = 'commands', boolean createByDefaultPath = true) {
        File file = null
        try {
            file = createByDefaultPath ? File.createTempFile(fileName, '.txt') : new File("${fileName}")

            fileRows.each {
                file << "${it.coordinate.name} ${it.delay}\n"
            }
        } catch (e) {
            logService.writeLog(this, "TXT generate error: $e".toString(), LogStatus.ERROR)
        }

        return file
    }
}
