package ru.armbot.controller

import io.micronaut.http.annotation.Get
import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.PathVariable
import io.micronaut.http.HttpResponse
import io.micronaut.core.io.ResourceResolver
import io.micronaut.http.server.types.files.StreamedFile
import io.micronaut.http.MediaType

@Controller("/")
class MainController {

    MainController() {}

    @Get(value = "{path:[^\\.]*}", consumes = MediaType.TEXT_HTML)
    HttpResponse gui(@PathVariable Optional path) {
        StreamedFile indexFile = new StreamedFile(new ResourceResolver().getResource("classpath:public/index.html").get())
        return HttpResponse.ok(indexFile)
    }
}
