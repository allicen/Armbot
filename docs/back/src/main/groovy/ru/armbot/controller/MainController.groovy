package ru.armbot.controller

import io.micronaut.core.annotation.Creator
import io.micronaut.http.annotation.Get
import io.micronaut.http.annotation.Controller

import org.slf4j.Logger
import org.slf4j.LoggerFactory
import ru.armbot.repository.SessionStateRepository

@Controller("/test")
class MainController {

    private final Logger logger = LoggerFactory.getLogger(this.getClass())

    @Creator
    MainController() { }

    @Get()
    static String gui() {
        return "forward:/public/index.html"
    }
}