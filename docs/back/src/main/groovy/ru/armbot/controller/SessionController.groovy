package ru.armbot.controller

import io.micronaut.http.MediaType
import io.micronaut.http.annotation.Post
import ru.armbot.domain.Coordinate
import ru.armbot.domain.Image
import ru.armbot.domain.LaunchFileRow
import ru.armbot.domain.Settings
import ru.armbot.domain.WorkOption
import ru.armbot.dto.FileRowDto
import ru.armbot.dto.ResponseDto
import ru.armbot.domain.ResponseStatus
import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Get
import jakarta.inject.Inject
import ru.armbot.domain.SessionState
import ru.armbot.dto.SessionStateDto
import ru.armbot.repository.CoordinateRepository
import ru.armbot.repository.ImageRepository
import ru.armbot.repository.LaunchFileRowRepository
import ru.armbot.repository.SessionStateRepository
import ru.armbot.repository.SettingsRepository
import ru.armbot.service.JsonService
import ru.armbot.utils.EnumService

@Controller("/session")
class SessionController {

    @Inject SessionStateRepository sessionStateRepository
    @Inject CoordinateRepository coordinateRepository
    @Inject ImageRepository imageRepository
    @Inject SettingsRepository settingsRepository
    @Inject LaunchFileRowRepository launchFileRowRepository
    @Inject JsonService jsonService


    @Get(value = '/get')
    def get() {

        List<SessionState> sessionList = sessionStateRepository.list()

        if (sessionList.size() == 0) {
            return new ResponseDto(status: ResponseStatus.NO_SESSION)
        }

        SessionStateDto sessionStateDto = createModel()
        return new ResponseDto(status: ResponseStatus.SUCCESS, details: sessionStateDto)
    }


    @Get(value = "/remove")
    def remove() {
        try {
            imageRepository.deleteAll()
            coordinateRepository.deleteAll()
            sessionStateRepository.deleteAll()
            settingsRepository.deleteAll()

            return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Сеанс успешно завершен')
        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERROR_REMOVE_SESSION', message: 'При завершении сеанса произошла ошибка')
        }
    }


    @Get(value = "/export", produces = MediaType.APPLICATION_JSON_STREAM)
    def export() {
        List<SessionState> sessionList = sessionStateRepository.list()

        if (sessionList.size() == 0) {
            return jsonService.sessionJsonFile(new SessionStateDto())
        }

        SessionStateDto sessionStateDto = createModel()
        return jsonService.sessionJsonFile(sessionStateDto)
    }



    @Post(value = "/import")
    def importFile(SessionStateDto session) {

        List<String> errors = []

        if (session.image) {
            Image image = new Image(name: session.image.name,
                    imageByte: session.image.imageByte,
                    contentType: session.image.contentType,
                    imagePositionX: session.image.imagePositionX,
                    imagePositionY: session.image.imagePositionY,
                    imageWidthPx: session.image.imageWidthPx,
                    canEdit: session.image.canEdit)

            try {
                imageRepository.save(image)
            } catch (e) {
                println(e)
                errors += 'При загрузке картинки возникли ошибки'
            }
        }

        if (session.coordinateList) {
            session.coordinateList.each { item ->
                Coordinate coordinate = new Coordinate(name: item.name, x: item.x, y: item.y, z: item.z)
                try {
                    coordinateRepository.save(coordinate)
                } catch (e) {
                    println(e)
                    errors.push("При загрузке координаты с ID=${item.id} (${item.name} возникли ошибки)".toString())
                }
            }

            session.launchFileRowList?.each { item ->
                Coordinate coordinate = coordinateRepository.list().find {it.id == item.coordinateId}
                if (!coordinate) {
                    errors.push("Строка файла запуска не может быть сохранена. Причина: отсутствует координата с ID=${item.coordinateId}".toString())
                } else {
                    try {
                        LaunchFileRow launchFileRow = new LaunchFileRow(coordinate: coordinate, delay: item.delay, sortOrder: item.sortOrder)
                        launchFileRowRepository.save(launchFileRow)
                    } catch (e) {
                        println(e)
                        errors.push("При сохранении строки для файла запуска (coordinateId=${item.coordinateId}, delay=${item.delay}, sortOrder=${item.sortOrder}) возникли ошибки".toString())
                    }
                }
            }
        }

        if (session.settings) {
            Settings settings = new Settings(cursorArea: session.settings.cursorArea)

            try {
                settingsRepository.save(settings)
            } catch (e) {
                println(e)
                errors.push('При сохранении настроек возникли ошибки')
            }
        }


        SessionState sessionState = new SessionState(workOption: EnumService.getValue(WorkOption, session.workOption))
        if (imageRepository.list().size() > 0) {
            sessionState.image = imageRepository.list()[0]
        }

        if (settingsRepository.list().size() > 0) {
            sessionState.settings = settingsRepository.list()[0]
        }

        try {
            sessionStateRepository.save(sessionState)
        } catch (e) {
            println(e)
            errors.push("При сохранениии суссии возникли ошибки")
        }

        if (errors.size() > 0) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERRORS_IMPORT_SESSION',
                    message: 'При импорте сессии возникли ошибки', details: errors)
        }

        return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Импорт сессии прошел успешно')
    }


    SessionStateDto createModel() {
        List<FileRowDto> fileRows = []
        launchFileRowRepository.list().each {
            fileRows += new FileRowDto(id: it.id, coordinateId: it.coordinate.id, delay: it.delay, sortOrder: it.sortOrder)
        }

        fileRows.sort { it.sortOrder }

        SessionState session = sessionStateRepository.list()[0]
        return new SessionStateDto(sessionId: session.id,
                workOption: session.workOption.id,
                image: session.image,
                settings: session.settings,
                coordinateList: coordinateRepository.list(),
                launchFileRowList: fileRows)
    }
}
