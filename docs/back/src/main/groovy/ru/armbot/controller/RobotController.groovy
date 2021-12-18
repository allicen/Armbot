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
        println('TEST!!!!!!!!!!!')
        def armbotPath = System.getenv('ARMBOT_PATH')
//        def proc = "${armbotPath}/scripts/armbot.sh start".execute()
//        def b = new StringBuffer()
//        proc.consumeProcessErrorStream(b)

        def sout = new StringBuilder(), serr = new StringBuilder()
        def proc = "${armbotPath}/scripts/armbot.sh stop".execute()
        proc.consumeProcessOutput(sout, serr)
        proc.waitForOrKill(1000)


        println('FINISH!!!!!!!!!!!')
//        println proc.text
//        println b.toString()

        return new ResponseDto(status: ResponseStatus.CONNECT)
    }
}
