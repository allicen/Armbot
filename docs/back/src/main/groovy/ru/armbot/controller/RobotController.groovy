package ru.armbot.controller

import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Get
import jakarta.inject.Inject
import ru.armbot.domain.LogStatus
import ru.armbot.domain.ResponseStatus
import ru.armbot.dto.ResponseDto
import ru.armbot.repository.CoordinateRepository
import ru.armbot.repository.LaunchFileRowRepository
import ru.armbot.service.LogService
import ru.armbot.service.TxtService


@Controller("/robot")
class RobotController {

    @Inject LogService logService
    @Inject TxtService txtService
    @Inject CoordinateRepository coordinateRepository
    @Inject LaunchFileRowRepository launchFileRowRepository


    RobotController() { }

    @Get(value = '/run')
    def run() {
        String fileSuffix = Math.abs(new Random().nextInt()).toString()

        String homePath = System.getProperty('user.home')
        String robotLaunchPathShort = "armbot-info/launch/${fileSuffix}"
        String robotLaunchPath = "${homePath}/${robotLaunchPathShort}"

        File launchDirectory = new File(robotLaunchPath)
        if (!launchDirectory.exists()) {
            launchDirectory.mkdir()
        }

        String fileCommandsDescriptionName = "command_description.txt"
        String fileCommandsName = "commands.txt"

        String fileCommandsDescriptionPath = "${launchDirectory}/${fileCommandsDescriptionName}".toString()
        String fileCommandsPath = "${launchDirectory}/${fileCommandsName}".toString()

        File fileCommandsDescription = txtService.getCoordinateTxtFile(coordinateRepository.list(), "${fileCommandsDescriptionPath}", false)
        File fileCommands = txtService.getFileRowTxtFile(launchFileRowRepository.list(), "${fileCommandsPath}", false)

        String mess

        try {
            fileCommandsDescription.createNewFile()
            logService.writeLog(this, "Файл ${fileCommandsDescriptionPath} успешно создан".toString(), LogStatus.INFO)
            fileCommands.createNewFile()
            logService.writeLog(this, "Файл ${fileCommandsPath} успешно создан".toString(), LogStatus.INFO)
            return new ResponseDto(status: ResponseStatus.SUCCESS,
                    details: ['command_description': "/home/${robotLaunchPathShort}/${fileCommandsDescriptionName}".toString(),
                              'commands': "/home/${robotLaunchPathShort}/${fileCommandsName}".toString(),
                              'path': "${fileSuffix}".toString()],
                    message: 'Файлы запуска успешно сформированы')
        } catch (e) {
            mess = "При формировании файлов запуска ${fileCommandsDescription}, ${fileCommands} возникли ошибки".toString()
            logService.writeLog(this, "${mess}: $e".toString(), LogStatus.ERROR)
        }

        return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'SAVE_FILES_ERROR', message: mess)
    }
}
