package ru.armbot.controller

import io.micronaut.core.annotation.Creator
import io.micronaut.http.annotation.Get
import io.micronaut.http.annotation.Controller

import org.slf4j.Logger
import org.slf4j.LoggerFactory
import ru.armbot.repository.SessionStateRepository

import io.micronaut.http.HttpResponse
import jakarta.inject.Inject
import io.micronaut.core.io.ResourceResolver
import io.micronaut.http.server.types.files.StreamedFile
import io.micronaut.http.MediaType

@Controller("/")
class MainController {

    private final Logger logger = LoggerFactory.getLogger(this.getClass())

    MainController() {}

    @Get(value = '/{/path:[.]}', consumes = MediaType.TEXT_HTML)
    HttpResponse gui(Optional<String> path) {
        StreamedFile indexFile = new StreamedFile(new ResourceResolver().getResource("classpath:public/index.html").get())
        return HttpResponse.ok(indexFile)
    }
}
