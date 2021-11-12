package ru.armbot.controller

import io.micronaut.http.MediaType
import io.micronaut.http.annotation.Controller
import io.micronaut.http.annotation.Get
import io.micronaut.http.annotation.Part
import io.micronaut.http.annotation.Post
import jakarta.inject.Inject
import ru.armbot.domain.Coordinate
import ru.armbot.domain.LaunchFileRow
import ru.armbot.domain.ResponseStatus
import ru.armbot.dto.ResponseDto
import ru.armbot.repository.CoordinateRepository
import ru.armbot.repository.LaunchFileRowRepository
import ru.armbot.service.TxtService

@Controller("/file")
class LaunchFileRowController {

    @Inject LaunchFileRowRepository launchFileRowRepository
    @Inject CoordinateRepository coordinateRepository
    @Inject TxtService txtService

    LaunchFileRowController() {}

    @Get(value = "/save/{coordinateId}")
    def save(long coordinateId) {

        Coordinate coordinate = coordinateRepository.list().find {it.id == coordinateId }

        if (!coordinate) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'COORDINATE_NOT_FOUND', message: 'Координата не найдена')
        }
        LaunchFileRow fileRow = new LaunchFileRow(coordinate: coordinate, sortOrder: launchFileRowRepository.list().size())

        try {
            launchFileRowRepository.save(fileRow)
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Строка успешно добавлена')
        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'COORDINATE_NOT_SAVE', message: 'Ошибка при сохранении координаты')
        }
    }

    @Get(value = "/remove/{fileRowId}")
    def remove(long fileRowId) {
        LaunchFileRow fileRow = findRow(fileRowId)

        if (!fileRow) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'FILE_ROW_NOT_FOUND', message: 'Строка файла не найдена')
        }

        try {
            launchFileRowRepository.delete(fileRow)
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Строка успешно удалена')

        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERROR_REMOVE_FILE_ROW', message: 'При удалении строки возникла ошибка')
        }
    }


    @Post(value = "/setDelay", consumes = MediaType.MULTIPART_FORM_DATA)
    def setDelay(@Part Long fileRowId, @Part int delay) {
        LaunchFileRow fileRow = launchFileRowRepository.list().find {it.id == fileRowId }

        if (!fileRow) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'FILE_ROW_NOT_FOUND', message: 'Строка файла не найдена')
        }

        if (delay >= 0) {
            fileRow.delay = delay
        }

        try {
            launchFileRowRepository.update(fileRow)
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Строка успешно обновлена')

        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERROR_UPDATE_FILE_ROW',
                    message: 'При обновлении строки врзникла ошибка')
        }
    }


    @Post(value = "/sort", consumes = MediaType.MULTIPART_FORM_DATA)
    def sort(@Part Long rowIdFirst, @Part int sortOrderFirst, @Part Long rowIdSecond, @Part int sortOrderSecond) {
        LaunchFileRow firstRow = findRow(rowIdFirst)
        LaunchFileRow secondRow = findRow(rowIdSecond)

        if (!firstRow || !secondRow) {
            return new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'FILE_ROW_NOT_FOUND', message: 'Не найдена строка файла')
        }

        firstRow.sortOrder = sortOrderFirst
        secondRow.sortOrder = sortOrderSecond

        try {
            launchFileRowRepository.update(firstRow)
            launchFileRowRepository.update(secondRow)
            return new ResponseDto(status: ResponseStatus.SUCCESS, message: 'Порядок сортировки успешно сохранен')

        } catch (e) {
            println(e)
            return new ResponseDto(status: ResponseStatus.ERROR,
                    errorCode: 'ERROR_SET_SORT_ORDER', message: 'При сохранении сортировки произошла ошибка')
        }
    }


    @Get(value = "/txt")
    def exportCoordinatesTxt() {
        def list = launchFileRowRepository.list()
        return txtService.fileRowTxtFile(list)
    }


    LaunchFileRow findRow(long id) {
        return launchFileRowRepository.list().find {it.id == id }
    }
}
