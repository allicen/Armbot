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
                image: session.image,
                settings: session.settings,
                coordinateList: coordinateRepository.list())

        return new ResponseDto(status: ResponseStatus.SUCCESS, details: sessionStateDto)
    }

    @Post(value = "/save", consumes = MediaType.MULTIPART_FORM_DATA)
    def save(@Part byte[] file,
             @Part String name,
             @Part String contentType,
             @Part Integer imagePositionX,
             @Part Integer imagePositionY,
             @Part Integer imageWidthPx,
             @Part Integer cursorArea) {

        List<SessionState> sessionList = sessionStateRepository.list()
        SessionState session

        try {

            if (sessionList.size() == 0) {
                session = new SessionState()
            } else {
                session = sessionList.get(0)
            }

            if (file && contentType) {
                Image image = new Image(name: name,
                        contentType: contentType,
                        imageByte: file,
                        imagePositionX: imagePositionX,
                        imagePositionY: imagePositionY)

                if (imageWidthPx > 0) {
                    image.imageWidthPx = imageWidthPx
                }

                if (cursorArea > 0) {
                    Settings settings = new Settings(cursorArea: cursorArea)
                    settingsRepository.save(settings)
                }

                imageRepository.save(image)
                session.image = image
            }

            if (sessionList.size() == 0) {
                sessionStateRepository.save(session)
            } else {
                sessionStateRepository.update(session)
            }

            return new ResponseDto(status: ResponseStatus.SUCCESS, details: getSession(session))
        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: "ERROR_SAVE_SESSION", message: 'Ошибка сохранения сессии')
        }
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
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Сеанс успешно завершен')
        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERROR_REMOVE_SESSION', message: 'При завершении сеанса произошла ошибка')
        }
    }

}
