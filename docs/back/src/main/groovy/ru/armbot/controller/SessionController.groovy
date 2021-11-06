package ru.armbot.controller


import ru.armbot.dto.ResponseDto
import ru.armbot.domain.ResponseStatus
import ru.armbot.dto.SessionStateDto
import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Get
import io.micronaut.http.annotation.Part
import io.micronaut.http.annotation.Post
import jakarta.inject.Inject
import ru.armbot.domain.Image
import ru.armbot.domain.SessionState
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
            return new ResponseDto(status: ResponseStatus.SUCCESS)
        }

        SessionStateDto session =
                new SessionStateDto(sessionState: sessionList.get(0), coordinateList: coordinateRepository.list())
        return new ResponseDto(status: ResponseStatus.SUCCESS, details: session)
    }

    @Post(value = "/save")
    def save(@Part int imageSize, @Part int imagePositionX, @Part int imagePositionY) {

        List<SessionState> sessionList = sessionStateRepository.list()

        if (imageRepository.list().size() == 0) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: "IMAGE_NOT_FOUND", message: 'Картинка не найдена')
        }

        Image image = imageRepository.list()[0]

        try {
            SessionState session

            if (sessionList.size() == 0) {
                session = new SessionState(imageId: image.id, imageSize: imageSize, imagePositionX: imagePositionX, imagePositionY: imagePositionY)
                sessionStateRepository.save(session)
            } else {
                session = sessionList.get(0)
                session.imageSize = imageSize
                session.imagePositionX = imagePositionX
                session.imagePositionY = imagePositionY
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
