package ru.armbot.controller

import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Get
import jakarta.inject.Inject
import ru.armbot.domain.ResponseStatus
import ru.armbot.dto.ResponseDto
import ru.armbot.service.LogService


@Controller("/robot")
class RobotController {

    @Inject LogService logService

    RobotController() { }

    @Get(value = '/run')
    def run() {





        return new ResponseDto(status: ResponseStatus.CONNECT)
    }
}
