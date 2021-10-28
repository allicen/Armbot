package ru.armbot.service

import io.micronaut.context.annotation.DefaultImplementation
import io.micronaut.http.server.types.files.SystemFile

import io.micronaut.core.annotation.NonNull
import ru.armbot.domain.Coordinate
import javax.validation.Valid
import javax.validation.constraints.NotNull;


@DefaultImplementation(CoordinateExcelService.class)
interface CoordinateExcelServiceImpl {
    String SHEET_NAME = "Coordinates"
    String HEADER_NAME = "Название координаты"
    String HEADER_X = "Координата X"
    String HEADER_Y = "Координата Y"
    String HEADER_Z = "Координата Z"
    String HEADER_EXCEL_FILE_SUFIX = ".xlsx"
    String HEADER_EXCEL_FILE_PREFIX = "Coordinate"
    String HEADER_EXCEL_FILENAME = HEADER_EXCEL_FILE_PREFIX + HEADER_EXCEL_FILE_SUFIX

    @NonNull
    SystemFile excelFileFromCoordinates(@NonNull @NotNull List<@Valid Coordinate> coordinateList);
}
