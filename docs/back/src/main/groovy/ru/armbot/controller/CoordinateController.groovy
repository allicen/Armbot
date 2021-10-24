package ru.armbot.controller

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

@Controller("/coordinate")
class CoordinateController {
    private final Logger logger = LoggerFactory.getLogger(this.getClass())

    @Inject CoordinateRepository coordinateRepository
    @Inject CoordinateExcelService coordinateExcelService;

    CoordinateController() {}

    @Post(value = "/save")
    def saveCoordinates(@Body List<Coordinate> coordinateList) {

        coordinateList.each {
            try {
                coordinateRepository.save(it)
            } catch (e) {
                def message = "Ошибка сохранения координаты ${it.name}. ".toString()
                println(message + e)
            }
        }

        return HttpStatus.OK
    }


    @Produces(value = "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet")
    @Get(value = "/excel")
    def saveCoordinates() {
        def list = coordinateRepository.list()
        return coordinateExcelService.excelFileFromCoordinates(list);
    }
}
