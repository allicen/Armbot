package ru.armbot.controller

import dto.ResponseDto
import dto.ResponseStatus
import io.micronaut.http.HttpStatus
import io.micronaut.http.annotation.Body
import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Get
import io.micronaut.http.annotation.Post
import io.micronaut.http.annotation.Produces
import jakarta.inject.Inject
import org.slf4j.Logger;
import org.slf4j.LoggerFactory
import ru.armbot.domain.Coordinate
import ru.armbot.repository.CoordinateRepository
import ru.armbot.service.CoordinateExcelService
import ru.armbot.service.CoordinateTxtService

@Controller("/coordinate")
class CoordinateController {
    private final Logger logger = LoggerFactory.getLogger(this.getClass())
    private final String PREFIX = 'coordinate'

    CoordinateController() { }

    @Inject CoordinateRepository coordinateRepository
    @Inject CoordinateExcelService coordinateExcelService
    @Inject CoordinateTxtService coordinateTxtService

    @Post(value = "/save")
    def save(@Body Coordinate coordinate) {
        int nameTail = coordinateRepository.list().size()
        String name = "${PREFIX}-${nameTail}"

        while (nameExist(name)) {
            nameTail++
            name = "${PREFIX}-${nameTail}"
        }

        coordinate.id = null
        coordinate.name = name

        try {
            coordinateRepository.save(coordinate)
            return [status: 'SUCCESS', coordinateName: name]

        } catch (e) {
            def message = "Ошибка сохранения координаты ${coordinate.name}. ".toString()
            println(message + e)
        }

        return HttpStatus.INTERNAL_SERVER_ERROR
    }

    @Post(value = "/update")
    def update(@Body Coordinate coordinate) {
        if (!coordinateRepository.list().find { it.id == coordinate.id} ) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'NOT_FOUND', message: 'Координата не найдена')
        }

        try {
            coordinateRepository.update(coordinate)
            return new ResponseDto(status: ResponseStatus.SUCCESS)
        } catch (e) {
            println("Ошибка обновления координаты, ${e}")
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'UPDATE_ERROR', message: 'Ошибка обновления координаты')
        }
    }

    @Get(value = "/removeAll")
    def removeAll() {
        try {
            coordinateRepository.deleteAll()
            return new ResponseDto(status: ResponseStatus.SUCCESS)
        } catch (e) {
            println("Ошибка удаления координат, ${e}")
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'DELETE_ERROR', message: 'Ошибка удаления координат')
        }
    }

    @Get(value = "/remove/{id}")
    def remove(long id) {

        Coordinate coordinate = coordinateRepository.list().find { it.id == id }
        if (!coordinate ) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'NOT_FOUND', message: 'Координата не найдена')
        }

        try {
            coordinateRepository.delete(coordinate)
            return new ResponseDto(status: ResponseStatus.SUCCESS)
        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'DELETE_ERROR', message: 'Ошибка удаления координаты')
        }
    }

    @Produces(value = "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet")
    @Get(value = "/excel")
    def saveCoordinates() {
        def list = coordinateRepository.list()
        return coordinateExcelService.excelFileFromCoordinates(list);
    }

    @Get(value = "/txt")
    def exportCoordinatesTxt() {
        def list = coordinateRepository.list()
        return coordinateTxtService.txtFile(list)
    }


    boolean nameExist(name) {
        return coordinateRepository.list().find{it.name == name }
    }
}
