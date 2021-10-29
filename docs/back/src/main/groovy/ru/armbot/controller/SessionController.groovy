package ru.armbot.controller


import dto.ResponseDto
import dto.ResponseStatus
import dto.SessionStateDto
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


        SessionStateDto session =
                new SessionStateDto(sessionState: sessionStateRepository.list()?.get(0) ?: null, coordinateList: coordinateRepository.list())
        return new ResponseDto(status: ResponseStatus.SUCCESS, details: session)
    }

    @Post(value = "/save")
    def save(@Part long imageId, @Part int imageSize, @Part int imagePositionX, @Part int imagePositionY) {

        List<SessionState> sessionList = sessionStateRepository.list()
        Image image = imageRepository.list().find { it.id == imageId }

        if (!image) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: "IMAGE_NOT_FOUND", message: 'Картинка не найдена')
        }

        try {
            SessionState session

            if (sessionList.size() == 0) {
                session = new SessionState(imageId: imageId, imageSize: imageSize, imagePositionX: imagePositionX, imagePositionY: imagePositionY)
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
            sessionStateRepository.deleteAll()
            return new ResponseDto(status: ResponseStatus.SUCCESS)
        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERROR_REMOVE_SESSION', message: 'Ошибка удаления сессии')
        }
    }

}
