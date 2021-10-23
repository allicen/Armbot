package ru.armbot.controller

import io.micronaut.http.HttpStatus
import io.micronaut.http.MediaType
import io.micronaut.http.annotation.Body
import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Get
import io.micronaut.http.annotation.Part
import io.micronaut.http.annotation.PathVariable
import io.micronaut.http.annotation.Post
import jakarta.inject.Inject
import ru.armbot.domain.Image
import ru.armbot.repository.ImageRepository

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


@Controller("/image")
class ImageController {

    private final Logger logger = LoggerFactory.getLogger(this.getClass())

    private List<String> accessMimeType = ['image/jpeg', 'image/gif', 'image/png', 'image/svg+xml', 'image/tiff']

    @Inject ImageRepository imageRepository

    ImageController() {}

    @Get(value = "/getImage")
    def getImage() {
        def images = imageRepository.list()

        if (images.isEmpty()) {
            return [status : 'ERROR',
                    code: 'IMAGE_LIST_EMPTY',
                    message: 'Не найдено ни одного изображения.']
        }

        def image = images.pop()

        // Если в базе больше 1 картинки, оставляем последнюю загруженную
        if (images.size() > 1) {
            for (i in 0..< images.size()-1) {
                imageRepository.deleteById(images[i].id)
            }
        }

        return [status: 'OK', image: image]
    }


    @Post(value = "/upload", consumes = MediaType.MULTIPART_FORM_DATA)
    def uploadImage(@Part byte[] file, @Part String name, @Part String contentType) {

        if (!accessMimeType.contains(contentType)) {
            def message = "Тип файла ${contentType} не поддерживается! Разрешены: ${accessMimeType.join(', ')}".toString()
            return [status : 'ERROR',
                    code: 'INVALID_MIME_TYPE',
                    message: message]
        }

        // Может быть загружена только 1 картинка
        if (!imageRepository.list().isEmpty()) {
            imageRepository.deleteAll()
        }

        Image image = new Image(file, contentType, name)

        try {
            imageRepository.save(image)
            return [status: 'OK', message: 'Изображение успешно загружено']

        } catch (e) {
            String message = 'При сохранении изображения произошла ошибка'
            println("${message}: " + e)
            return [status : 'ERROR',
                    code: 'ERROR_SAVE_IMAGE',
                    message: 'При сохранении изображения произошла ошибка']
        }
    }

    @Get(value = "/removeImage/")
    def removeImage() {

        try {
            imageRepository.deleteAll()
            [status: 'OK', message: 'Изображение успешно удалено']

        } catch (e) {
            String message = "При удалении изображения произошла ошибка. "
            println(message + e)
            return [status : 'ERROR',
                    code: 'ERROR_REMOVE_IMAGE',
                    message: message]
        }
    }
}
