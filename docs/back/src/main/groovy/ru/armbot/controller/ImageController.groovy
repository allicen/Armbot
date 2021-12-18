package ru.armbot.controller

import io.micronaut.http.MediaType
import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Part
import io.micronaut.http.annotation.Post
import jakarta.inject.Inject
import ru.armbot.domain.Image
import ru.armbot.domain.LogStatus
import ru.armbot.domain.ResponseStatus
import ru.armbot.domain.SessionState
import ru.armbot.domain.WorkOption
import ru.armbot.dto.ResponseDto
import ru.armbot.repository.CoordinateRepository
import ru.armbot.repository.ImageRepository

import org.slf4j.Logger
import org.slf4j.LoggerFactory
import ru.armbot.repository.SessionStateRepository
import ru.armbot.service.LogService
import ru.armbot.service.SessionService


@Controller("/image")
class ImageController {
    private List<String> accessMimeType = ['image/jpeg', 'image/gif', 'image/png', 'image/svg+xml', 'image/tiff']

    @Inject ImageRepository imageRepository
    @Inject CoordinateRepository coordinateRepository
    @Inject SessionStateRepository sessionStateRepository
    @Inject SessionService sessionService
    @Inject LogService logService

    ImageController() {}


    @Post(value = "/upload", consumes = MediaType.MULTIPART_FORM_DATA)
    def uploadImage(@Part byte[] file, @Part String name, @Part String contentType) {

        if (!accessMimeType.contains(contentType)) {
            def message = "Тип файла ${contentType} не поддерживается! Разрешены: ${accessMimeType.join(', ')}".toString()
            return new ResponseDto(status : 'ERROR', errorCode: 'INVALID_MIME_TYPE', message: message)
        }

        // Очищаем незавершенные сессии (если такие есть)
        sessionService.clearAll()

        SessionState sessionState = new SessionState(workOption: WorkOption.UPLOAD_IMAGE)

        try {
            sessionStateRepository.save(sessionState)
            logService.writeLog(this, 'Сеанс успешно сохранена')
        } catch (e) {
            String mess = 'Сеанс не создана'
            logService.writeLog(this, ("$mess: $e").toString(), LogStatus.ERROR)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'SESSION_NOT_SAVE', message: mess)
        }

        // Может быть загружена только 1 картинка
        if (!imageRepository.list().isEmpty()) {
            imageRepository.deleteAll()
        }

        Image image = new Image(imageByte: file, contentType: contentType, name: name, sessionState: sessionState)

        try {
            imageRepository.save(image)
            String mess = 'Изображение успешно загружено'
            logService.writeLog(this, mess)
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: mess)
        } catch (e) {
            String mess = 'При сохранении изображения произошла ошибка'
            logService.writeLog(this, ("$mess: $e").toString(), LogStatus.ERROR)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERROR_SAVE_IMAGE', message: mess)
        }
    }


    @Post(value = "/setImageDetails", consumes = MediaType.MULTIPART_FORM_DATA)
    def setImageDetails(@Part Integer imagePositionX, @Part Integer imagePositionY, @Part Integer imageWidthPx, @Part boolean canEdit) {

        List<Image> images = imageRepository.list()
        if (images.size() == 0) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'IMAGE_NOT_FOUND', message: 'Картинка не найдена')
        }

        Image image = images[0]
        image.imagePositionX = imagePositionX
        image.imagePositionY = imagePositionY
        image.imageWidthPx = imageWidthPx
        image.canEdit = canEdit

        try {
            imageRepository.update(image)
            logService.writeLog(this, 'Изображение успешно обновлено')
            return new ResponseDto(status: ResponseStatus.SUCCESS)
        } catch (e) {
            String mess = 'Картинка не обновлена'
            logService.writeLog(this, ("$mess: $e").toString(), LogStatus.ERROR)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'IMAGE_NOT_UPDATE', message: mess)
        }
    }
}
