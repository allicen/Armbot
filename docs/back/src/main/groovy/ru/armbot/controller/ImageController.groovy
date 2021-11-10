package ru.armbot.controller

import io.micronaut.http.MediaType
import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Get
import io.micronaut.http.annotation.Part
import io.micronaut.http.annotation.Post
import jakarta.inject.Inject
import ru.armbot.domain.Image
import ru.armbot.domain.ResponseStatus
import ru.armbot.domain.SessionState
import ru.armbot.dto.ResponseDto
import ru.armbot.repository.CoordinateRepository
import ru.armbot.repository.ImageRepository

import org.slf4j.Logger
import org.slf4j.LoggerFactory
import ru.armbot.repository.SessionStateRepository


@Controller("/image")
class ImageController {

    private final Logger logger = LoggerFactory.getLogger(this.getClass())

    private List<String> accessMimeType = ['image/jpeg', 'image/gif', 'image/png', 'image/svg+xml', 'image/tiff']

    @Inject ImageRepository imageRepository
    @Inject CoordinateRepository coordinateRepository
    @Inject SessionStateRepository sessionStateRepository

    ImageController() {}


    @Post(value = "/upload", consumes = MediaType.MULTIPART_FORM_DATA)
    def uploadImage(@Part byte[] file, @Part String name, @Part String contentType) {

        if (!accessMimeType.contains(contentType)) {
            def message = "Тип файла ${contentType} не поддерживается! Разрешены: ${accessMimeType.join(', ')}".toString()
            return new ResponseDto(status : 'ERROR', errorCode: 'INVALID_MIME_TYPE', message: message)
        }

        // Может быть загружена только 1 картинка
        if (!imageRepository.list().isEmpty()) {
            imageRepository.deleteAll()
        }

        Image image = new Image(file, contentType, name)

        try {
            imageRepository.save(image)

            // После загрузки изображения создаем сессию
            SessionState sessionState = new SessionState(image: image)
            sessionStateRepository.save(sessionState)

            return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Изображение успешно загружено')

        } catch (e) {
            String message = 'При сохранении изображения произошла ошибка'
            println("${message}: " + e)

            // если сессия не создалась, но изображение загружено, удаляем его
            if (image.id) {
                imageRepository.deleteAll()
                return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'CREATE_SESSION_ERROR', message: message)
            }

            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERROR_SAVE_IMAGE', message: message)
        }
    }
}
