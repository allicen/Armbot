package ru.armbot.controller


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

@Controller("/session")
class SessionController {

    @Inject SessionStateRepository sessionStateRepository
    @Inject CoordinateRepository coordinateRepository
    @Inject ImageRepository imageRepository


    @Get(value = '/get')
    def get() {

        List<SessionState> sessionList = sessionStateRepository.list()

        if (sessionList.size() == 0) {
            return new ResponseDto(status: ResponseStatus.NO_SESSION)
        }

        SessionState session = sessionStateRepository.list()[0]
        SessionStateDto sessionStateDto = new SessionStateDto(sessionId: session.id,
                image: session.image,
                settings: session.settings,
                coordinateList: coordinateRepository.list())

        return new ResponseDto(status: ResponseStatus.SUCCESS, details: sessionStateDto)
    }

    @Post(value = "/save")
    def save() {

        List<SessionState> sessionList = sessionStateRepository.list()

        try {
            SessionState session

            if (sessionList.size() == 0) {
                session = new SessionState()
                sessionStateRepository.save(session)
            } else {
                session = sessionList.get(0)
                sessionStateRepository.update(session)
            }

            return new ResponseDto(status: ResponseStatus.SUCCESS)
        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: "ERROR_SAVE_SESSION", message: 'Ошибка сохранения сессии')
        }
    }



    @Get(value = "/remove")
    def remove() {
        try {
            imageRepository.deleteAll()
            coordinateRepository.deleteAll()
            sessionStateRepository.deleteAll()
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Сеанс успешно завершен')
        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERROR_REMOVE_SESSION', message: 'При завершении сеанса произошла ошибка')
        }
    }

}
