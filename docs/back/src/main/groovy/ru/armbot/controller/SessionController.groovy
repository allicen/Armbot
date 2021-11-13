package ru.armbot.controller

import io.micronaut.http.MediaType
import io.micronaut.http.annotation.Post
import ru.armbot.domain.Coordinate
import ru.armbot.domain.Image
import ru.armbot.domain.LaunchFileRow
import ru.armbot.domain.Settings
import ru.armbot.domain.SizeUnit
import ru.armbot.domain.WorkOption
import ru.armbot.dto.CoordinateDto
import ru.armbot.dto.FileRowDto
import ru.armbot.dto.ImageDto
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
import ru.armbot.service.SessionService
import ru.armbot.utils.EnumService

@Controller("/session")
class SessionController {

    @Inject SessionStateRepository sessionStateRepository
    @Inject CoordinateRepository coordinateRepository
    @Inject ImageRepository imageRepository
    @Inject SettingsRepository settingsRepository
    @Inject LaunchFileRowRepository launchFileRowRepository
    @Inject JsonService jsonService
    @Inject SessionService sessionService


    @Get(value = '/get')
    def get() {

        List<SessionState> sessionList = sessionStateRepository.list()

        if (sessionList.isEmpty()) {
            return new ResponseDto(status: ResponseStatus.NO_SESSION)
        }

        SessionStateDto sessionStateDto = createModel()
        return new ResponseDto(status: ResponseStatus.SUCCESS, details: sessionStateDto)
    }


    @Get(value = "/remove")
    def remove() {
        try {
            sessionService.clearAll()
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

        if (!session.image || !session.coordinateList) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'REQUIRED_FIELDS_NOT_FOUND',
                    message: 'Не найдены обязательные поля. В файле обязательно должна быть картинка или список координат.')
        }

        // Очищаем незавершенные сессии (если такие есть)
        // Может быть только 1 сессия
        if (!sessionStateRepository.list().isEmpty()) {
            sessionStateRepository.deleteAll()
        }

        SessionState sessionState = new SessionState(workOption: EnumService.getValue(WorkOption, session.workOption))
        try {
            sessionStateRepository.save(sessionState)
        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERROR_SAVE_SESSION', message: "При сохранениии суссии возникли ошибки")
        }

        List<String> errors = []

        if (session.image) {
            Image image = new Image(name: session.image.name,
                    imageByte: session.image.imageByte,
                    contentType: session.image.contentType,
                    imagePositionX: session.image.imagePositionX,
                    imagePositionY: session.image.imagePositionY,
                    imageWidthPx: session.image.imageWidthPx,
                    canEdit: session.image.canEdit,
                    sessionState: sessionState)

            try {
                imageRepository.save(image)
            } catch (e) {
                println(e)
                errors += 'При загрузке картинки возникли ошибки'
            }
        }

        if (session.settings) {
            Settings settings = new Settings(cursorArea: session.settings.cursorArea, sessionState: sessionState)
            try {
                settingsRepository.save(settings)
            } catch (e) {
                println(e)
                errors.push('При сохранении настроек возникла ошибка')
            }
        }

        if (session.coordinateList) {
            session.coordinateList.each { item ->
                Coordinate coordinate = new Coordinate(name: item.name, x: item.x, y: item.y, z: item.z, sessionState: sessionState)
                try {
                    coordinateRepository.save(coordinate)
                } catch (e) {
                    println(e)
                    errors.push("При загрузке координаты с ID=${item.id} (${item.name} возникли ошибки)".toString())
                }
            }

            session.launchFileRowList?.each { item ->
                Long min = coordinateRepository.list().id.min()
                int importMin = session.launchFileRowList.coordinateId.min()
                Long offset = min - importMin
                Coordinate coordinate = coordinateRepository.list().find {it.id == offset + item.coordinateId}
                if (!coordinate) {
                    errors.push("Строка файла запуска не может быть сохранена. Причина: отсутствует координата с ID=${item.coordinateId}".toString())
                } else {
                    try {
                        LaunchFileRow launchFileRow = new LaunchFileRow(coordinate: coordinate, delay: item.delay, sortOrder: item.sortOrder, sessionState: sessionState)
                        launchFileRowRepository.save(launchFileRow)
                    } catch (e) {
                        println(e)
                        errors.push("При сохранении строки для файла запуска (coordinateId=${item.coordinateId}, delay=${item.delay}, sortOrder=${item.sortOrder}) возникли ошибки".toString())
                    }
                }
            }
        }

        if (errors.size() > 0) {

            try {
                sessionService.clearAll()
            } catch (e) {
                println(e)
                errors.push('При очищениии созданной сессии произошли ошибки. Перезапустите систему. Некоторые функции могут работать некорректно!')
            }

            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERRORS_IMPORT_SESSION',
                    message: 'При импорте сессии возникли ошибки', details: errors)
        }

        return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Импорт сессии прошел успешно')
    }


    SessionStateDto createModel() {
        List<FileRowDto> fileRows = []
        launchFileRowRepository.list().each {
            fileRows += new FileRowDto(id: it.id, coordinateId: it.coordinate?.id, delay: it.delay, sortOrder: it.sortOrder)
        }

        fileRows.sort { it.sortOrder }

        Image image = imageRepository.list()?.getAt(0)
        ImageDto imageDto = new ImageDto(
                name: image?.name,
                contentType: image?.contentType,
                imageByte: image?.imageByte,
                canEdit: image?.canEdit,
                imageWidthPx: image?.imageWidthPx,
                imagePositionX: image?.imagePositionX,
                imagePositionY: image?.imagePositionY
        )

        List<CoordinateDto> coordinateList = []
        coordinateRepository.list()?.each {
            coordinateList.push(new CoordinateDto(id: it.id, name: it.name, x: it.x, y: it.y, z: it.z, unit: it.unit))
        }

        SessionState session = sessionStateRepository.list()[0]
        return new SessionStateDto(sessionId: session.id,
                workOption: session.workOption.id,
                image: imageDto.imageByte.size() > 0 ? imageDto : null,
                settings: settingsRepository.list()?.getAt(0),
                coordinateList: coordinateList,
                launchFileRowList: fileRows)
    }
}
