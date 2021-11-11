package ru.armbot.controller

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

@Controller("/session")
class SessionController {

    @Inject SessionStateRepository sessionStateRepository
    @Inject CoordinateRepository coordinateRepository
    @Inject ImageRepository imageRepository
    @Inject SettingsRepository settingsRepository
    @Inject LaunchFileRowRepository launchFileRowRepository


    @Get(value = '/get')
    def get() {

        List<SessionState> sessionList = sessionStateRepository.list()

        if (sessionList.size() == 0) {
            return new ResponseDto(status: ResponseStatus.NO_SESSION)
        }

        List<FileRowDto> fileRows = []
        launchFileRowRepository.list().each {
            fileRows += new FileRowDto(id: it.id, coordinateId: it.coordinate.id, delay: it.delay, sortOrder: it.sortOrder)
        }

        fileRows.sort { it.sortOrder }

        SessionState session = sessionStateRepository.list()[0]
        SessionStateDto sessionStateDto = new SessionStateDto(sessionId: session.id,
                workOption: session.workOption.id,
                image: session.image,
                settings: session.settings,
                coordinateList: coordinateRepository.list(),
                launchFileRowList: fileRows)

        return new ResponseDto(status: ResponseStatus.SUCCESS, details: sessionStateDto)
    }



    private static SessionStateDto getSession(session) {
        return new SessionStateDto(image: session.image)
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

}
