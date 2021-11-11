package ru.armbot.controller

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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory
import ru.armbot.domain.Coordinate
import ru.armbot.repository.CoordinateRepository
import ru.armbot.repository.SessionStateRepository
import ru.armbot.service.CoordinateExcelService
import ru.armbot.service.CoordinateService
import ru.armbot.service.txtService

import java.nio.charset.StandardCharsets
import java.time.ZonedDateTime

@Controller("/coordinate")
class CoordinateController {
    private final Logger logger = LoggerFactory.getLogger(this.getClass())

    private List<String> accessMimeType = ['text/plain']

    CoordinateController() { }

    @Inject CoordinateService coordinateService
    @Inject CoordinateRepository coordinateRepository
    @Inject CoordinateExcelService coordinateExcelService
    @Inject txtService txtService
    @Inject SessionStateRepository sessionStateRepository

    @Post(value = "/save")
    def save(@Body Coordinate coordinate) {
        coordinate.id = null
        coordinate.name = coordinateService.generateName()
        coordinate.timeCreate = ZonedDateTime.now()

        try {
            coordinateRepository.save(coordinate)
            return new ResponseDto(status: 'SUCCESS', details: [coordinate: coordinate])

        } catch (e) {
            def message = "Ошибка сохранения координаты ${coordinate.name}. ".toString()
            println(message + e)
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

        try {
            coordinateRepository.update(coordinate)
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Координата обновлена!')
        } catch (e) {
            println("Ошибка обновления координаты, ${e}")
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'UPDATE_ERROR', message: 'Ошибка обновления координаты')
        }
    }

    @Get(value = "/removeAll")
    def removeAll() {
        try {
            coordinateRepository.deleteAll()
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Координаты успешно удалены')
        } catch (e) {
            println("Ошибка удаления координат, ${e}")
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'DELETE_ERROR', message: 'Ошибка удаления координат')
        }
    }

    @Get(value = "/remove/{id}")
    def remove(long id) {

        Coordinate coordinate = coordinateRepository.list().find { it.id == id }
        if (!coordinate ) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'NOT_FOUND', message: 'Координата не найдена')
        }

        try {
            coordinateRepository.delete(coordinate)
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Координата удалена!')
        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'DELETE_ERROR', message: 'Ошибка удаления координаты')
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

        def result = [success: 0, savedCoordinates: [], errorDetails: []]

        data.split('\n').eachWithIndex {row, i ->
            String index = (i + 1).toString()
            def rowData = row.split(':')
            if (rowData.size() == 2) {
                if (coordinateRepository.list().find {it.name == rowData[0]}) {
                    result.errorDetails += "Строка ${index} - название '${rowData[0].toString()}' уже существует".toString()
                } else {
                    def coordinates = rowData[1].split(' ')
                    if (coordinates.size() == 3) {
                        try {
                            Coordinate coordinate = new Coordinate(name: rowData[0].toString(),
                                    x: Double.parseDouble(coordinates[0]),
                                    y: Double.parseDouble(coordinates[1]),
                                    z: Double.parseDouble(coordinates[2]))
                            coordinateRepository.save(coordinate)
                            result.savedCoordinates += coordinate
                        } catch (e) {
                            result.errorDetails += "Строка ${index} - координата не была сохранена".toString()
                            println(e)
                        }
                    } else {
                        result.errorDetails += "Строка ${index} - содержит неверное количество координат".toString()
                    }
                }
            } else {
                result.errorDetails += "Строка ${index} - содержит не 1 двоеточие".toString()
            }
        }

        if (coordinateRepository.list().size() > 0 && sessionStateRepository.list().size() == 0) {
            SessionState sessionState = new SessionState(workOption: WorkOption.UPLOAD_COORDINATE_LIST)
            sessionStateRepository.save(sessionState)
        }

        if (result.errorDetails.size() > 0) {
            return new ResponseDto(status: ResponseStatus.ERROR,
                    errorCode: 'IMPORT_ERROR',
                    message: 'Координаты загружены с ошибками',
                    details: result)
        }

        return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Координаты успешно загружены', details: result)
    }

}
