package ru.armbot.controller

import io.micronaut.http.HttpStatus
import io.micronaut.http.MediaType
import io.micronaut.http.annotation.Body
import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Part
import io.micronaut.http.annotation.Post
import jakarta.inject.Inject
import ru.armbot.domain.Image
import ru.armbot.repository.ImageRepository


@Controller("/image")
class ImageController {

    @Inject ImageRepository imageRepository

    ImageController() {}

    @Post(value = "/")
    String getImage(@Body String image) {
        return HttpStatus.OK
    }


    @Post(value = "/upload", consumes = MediaType.MULTIPART_FORM_DATA)
    def uploadImage(@Part byte[] file, @Part String name, @Part String contentType) {
        Image image = new Image(file, contentType, name)
        imageRepository.save(image)
        return image //HttpStatus.OK
    }
}
