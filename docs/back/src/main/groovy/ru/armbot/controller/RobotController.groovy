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
        String robotLaunchDir = "${homePath}/armbot-info/launch" // Директория для хранения всех файлов запуска
        String robotLaunchPath = "${homePath}/${robotLaunchPathShort}" // Директория для хранения файлов для конкретного запуска

        // Создаем корневую директорию для записи файлов
        def createDirRes = createNewDirectory(robotLaunchDir)
        if (!createDirRes) {
            return createDirRes
        }

        // Создаем директорию для конкретного запуска
        createDirRes = createNewDirectory(robotLaunchPath)
        if (!createDirRes) {
            return createDirRes
        }

        String fileCommandsDescriptionName = "command_description.txt"
        String fileCommandsName = "commands.txt"

        String fileCommandsDescriptionPath = "${robotLaunchPath}/${fileCommandsDescriptionName}".toString()
        String fileCommandsPath = "${robotLaunchPath}/${fileCommandsName}".toString()

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


    def createNewDirectory(String dir) {
        File launchDirectory = new File(dir)
        if (!launchDirectory.exists()) {
            launchDirectory.mkdir()

            if (!launchDirectory.exists()) {
                String mess = "Не создана директория ${dir}".toString()
                logService.writeLog(this, mess, LogStatus.ERROR)
                return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'CREATE_DIRECTORY_ERROR', message: mess)
            }

            String mess = "Создана директория ${dir}".toString()
            logService.writeLog(this, mess, LogStatus.INFO)
        }

        return true
    }

    @Get(value = '/config')
    def getConfig() {
        String homePath = System.getProperty('user.home')
        String armbotConfigFile = "${homePath}/armbot-info/Armbot.txt"
        File configFile = new File(armbotConfigFile)

        if (configFile.exists()) {

            ArrayList<Map<String, String>> data = new ArrayList()

            configFile.eachLine {line ->
                String[] lineArr = line.split('=')
                if (line.length() > 0 && line[0] != '#' && line.indexOf('=') > -1 && lineArr.length == 2) {
                    String key = lineArr[0].trim()
                    String value = lineArr[1].trim()

                    Map<String, String> lineConfig = new HashMap<>()
                    lineConfig.put("key", key)
                    lineConfig.put("value", value)
                    data.add(lineConfig)
                }
            }

            logService.writeLog(this, "Файл с конфигами найден и обработан".toString(), LogStatus.INFO)
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: "Файл с конфигами найден и обработан", details: data)
        }

        logService.writeLog(this, "Файла с конфигами ${armbotConfigFile} не существует".toString(), LogStatus.ERROR)
        return new ResponseDto(status: ResponseStatus.SUCCESS, message: "Файла с конфигами ${armbotConfigFile} не существует".toString())
    }
}
