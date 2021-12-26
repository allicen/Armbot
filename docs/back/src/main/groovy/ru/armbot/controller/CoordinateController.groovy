package ru.armbot.controller

import ru.armbot.domain.LaunchFileRow
import ru.armbot.domain.LogStatus
import ru.armbot.domain.SessionState
import ru.armbot.domain.WorkOption
import ru.armbot.dto.ResponseDto
import io.micronaut.http.MediaType
import io.micronaut.http.annotation.Part
import ru.armbot.domain.ResponseStatus
import io.micronaut.http.HttpStatus
import io.micronaut.http.annotation.Body
import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Get
import io.micronaut.http.annotation.Post
import io.micronaut.http.annotation.Produces
import jakarta.inject.Inject
import ru.armbot.domain.Coordinate
import ru.armbot.repository.CoordinateRepository
import ru.armbot.repository.LaunchFileRowRepository
import ru.armbot.repository.SessionStateRepository
import ru.armbot.service.CoordinateExcelService
import ru.armbot.service.CoordinateService
import ru.armbot.service.LogService
import ru.armbot.service.TxtService

import java.nio.charset.StandardCharsets

@Controller("/coordinate")
class CoordinateController {

    private List<String> accessMimeType = ['text/plain']

    CoordinateController() { }

    @Inject CoordinateService coordinateService
    @Inject CoordinateRepository coordinateRepository
    @Inject CoordinateExcelService coordinateExcelService
    @Inject TxtService txtService
    @Inject SessionStateRepository sessionStateRepository
    @Inject LaunchFileRowRepository launchFileRowRepository
    @Inject LogService logService

    @Post(value = "/save")
    def save(@Body Coordinate coordinate) {
        coordinate.id = null
        coordinate.name = coordinateService.generateName()
        coordinate.sessionState = sessionStateRepository.list()?.getAt(0)

        try {
            coordinateRepository.save(coordinate)
            logService.writeLog(this, "Команда name=${coordinate.name} x=${coordinate.x} y=${coordinate.y} z=${coordinate.z} успешно сохранена".toString())
            return new ResponseDto(status: 'SUCCESS', details: [coordinate: coordinate])
        } catch (e) {
            def message = "Ошибка сохранения команды ${coordinate.name}. ".toString()
            logService.writeLog(this, ("$message: $e").toString(), LogStatus.ERROR)
        }

        return HttpStatus.INTERNAL_SERVER_ERROR
    }

    @Post(value = "/update")
    def update(@Body Coordinate coordinate) {

        Coordinate item = coordinateRepository.list().find { it.id == coordinate.id}

        if (!item ) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'NOT_FOUND', message: 'Координата не найдена')
        }

        if (coordinate.name != item.name && coordinateService.nameExist(coordinate.name)) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'INVALID_NAME',
                    message: "Координата с именем '${coordinate.name.toString()}' уже существует. ")
        }

        item.name = coordinate.name
        item.x = coordinate.x
        item.y = coordinate.y
        item.z = coordinate.z

        try {
            coordinateRepository.update(item)
            logService.writeLog(this, 'Команда успешно обновлена')
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Координата обновлена!')
        } catch (e) {
            String mess = 'Ошибка обновления команды'
            logService.writeLog(this, ("$mess: $e").toString(), LogStatus.ERROR)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'UPDATE_ERROR', message: mess)
        }
    }

    @Get(value = "/removeAll")
    def removeAll() {
        try {
            launchFileRowRepository.deleteAll()
            coordinateRepository.deleteAll()
            String mess = 'Команда успешно удалены'
            logService.writeLog(this, mess)
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: mess)
        } catch (e) {
            String mess = 'Ошибка удаления координат'
            logService.writeLog(this, ("$mess: $e"), LogStatus.ERROR)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'DELETE_ERROR', message: mess)
        }
    }

    @Get(value = "/remove/{id}")
    def remove(long id) {

        Coordinate coordinate = coordinateRepository.list().find { it.id == id }
        if (!coordinate ) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'NOT_FOUND', message: 'Координата не найдена')
        }

        LaunchFileRow launchFileRow = launchFileRowRepository.list().find {it.coordinate.id == id}

        try {
            if (launchFileRow) {
                launchFileRowRepository.delete(launchFileRow)
            }

            coordinateRepository.delete(coordinate)

            String mess = 'Команда удалена!'
            logService.writeLog(this, mess)
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: mess)
        } catch (e) {
            String mess = 'Ошибка удаления команды'
            logService.writeLog(this, ("$mess: $e").toString(), LogStatus.ERROR)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'DELETE_ERROR', message: mess)
        }
    }

    @Produces(value = "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet")
    @Get(value = "/excel")
    def saveCoordinates() {
        def list = coordinateRepository.list()
        return coordinateExcelService.excelFileFromCoordinates(list);
    }

    @Get(value = "/txt")
    def exportCoordinatesTxt() {
        def list = coordinateRepository.list()
        return txtService.coordinateTxtFile(list)
    }

    @Post(value = "/import", consumes = MediaType.MULTIPART_FORM_DATA)
    def uploadImage(@Part byte[] file, @Part String contentType) {

        if (!accessMimeType.contains(contentType)) {
            def message = "Тип файла ${contentType} не поддерживается! Разрешены: ${accessMimeType.join(', ')}".toString()
            return new ResponseDto(status : ResponseStatus.ERROR,
                    errorCode: 'INVALID_MIME_TYPE',
                    message: message)
        }

        String data = new String(file, StandardCharsets.UTF_8)

        if (data.size() == 0) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'EMPTY_FILE', message: 'Выбран пустой файл')
        }

        SessionState sessionState = new SessionState(workOption: WorkOption.UPLOAD_COORDINATE_LIST)
        try {
            sessionStateRepository.save(sessionState)
            logService.writeLog(this, 'Сеанс успешно сохранен')
        } catch (e) {
            String mess = 'При создании сессии произошла ошибка'
            logService.writeLog(this, ("$mess: $e"), LogStatus.ERROR)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERROR_CREATE_SESSION', message: mess)
        }

        def result = [success: 0, savedCoordinates: [], errorDetails: []]

        data.split('\n').eachWithIndex {row, i ->
            String index = (i + 1).toString()
            def rowData = row.split(':')
            if (rowData.size() == 2) {
                if (coordinateRepository.list().find {it.name == rowData[0]}) {
                    result.errorDetails += "Строка ${index} - название '${rowData[0].toString()}' уже существует".toString()
                } else {
                    def coordinates = rowData[1].split(' ')

                    // Координату Z ставим 0 по умолчанию
                    if (coordinates.size() == 2) {
                        coordinates += '0'
                    }

                    if (coordinates.size() == 3) {
                        try {
                            Coordinate coordinate = new Coordinate(name: rowData[0].toString(),
                                    x: Double.parseDouble(coordinates[0]) * 1000,
                                    y: Double.parseDouble(coordinates[1]) * 1000,
                                    z: Double.parseDouble(coordinates[2]) * 1000,
                                    sessionState: sessionState)
                            coordinateRepository.save(coordinate)
                            result.savedCoordinates += coordinate
                        } catch (e) {
                            String mess = "Строка ${index} - координата не была сохранена".toString()
                            result.errorDetails += mess
                            logService.writeLog(this, ("$mess: $e").toString(), LogStatus.ERROR)
                        }
                    } else {
                        result.errorDetails += "Строка ${index} - содержит неверное количество координат".toString()
                    }
                }
            } else {
                result.errorDetails += "Строка ${index} - содержит не 1 двоеточие".toString()
            }
        }

        if (result.errorDetails.size() > 0) {
            return new ResponseDto(status: ResponseStatus.ERROR,
                    errorCode: 'IMPORT_ERROR',
                    message: 'Команды загружены с ошибками',
                    details: result)
        }

        return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Команды успешно загружены', details: result)
    }

}
