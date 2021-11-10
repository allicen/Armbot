package ru.armbot.controller

import io.micronaut.core.annotation.Nullable
import io.micronaut.http.MediaType
import io.micronaut.http.annotation.Part
import ru.armbot.domain.Image
import ru.armbot.domain.Settings
import ru.armbot.dto.ResponseDto
import ru.armbot.domain.ResponseStatus
import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Get
import io.micronaut.http.annotation.Post
import jakarta.inject.Inject
import ru.armbot.domain.SessionState
import ru.armbot.dto.SessionStateDto
import ru.armbot.repository.CoordinateRepository
import ru.armbot.repository.ImageRepository
import ru.armbot.repository.SessionStateRepository
import ru.armbot.repository.SettingsRepository

@Controller("/session")
class SessionController {

    @Inject SessionStateRepository sessionStateRepository
    @Inject CoordinateRepository coordinateRepository
    @Inject ImageRepository imageRepository
    @Inject SettingsRepository settingsRepository


    @Get(value = '/get')
    def get() {

        List<SessionState> sessionList = sessionStateRepository.list()

        if (sessionList.size() == 0) {
            return new ResponseDto(status: ResponseStatus.NO_SESSION)
        }

        SessionState session = sessionStateRepository.list()[0]
        SessionStateDto sessionStateDto = new SessionStateDto(sessionId: session.id,
                workOption: session.workOption.id,
                image: session.image,
                settings: session.settings,
                coordinateList: coordinateRepository.list())

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
