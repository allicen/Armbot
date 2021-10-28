package ru.armbot.service

import builders.dsl.spreadsheet.builder.poi.PoiSpreadsheetBuilder;
import io.micronaut.core.annotation.NonNull;
import io.micronaut.http.HttpStatus;
import io.micronaut.http.exceptions.HttpStatusException;
import io.micronaut.http.server.types.files.SystemFile;

import jakarta.inject.Singleton
import ru.armbot.domain.Coordinate;

import javax.validation.Valid;
import javax.validation.constraints.NotNull;

import org.slf4j.Logger
import org.slf4j.LoggerFactory


@Singleton
class CoordinateExcelServiceImpl implements CoordinateExcelService {

    private static final Logger LOG = LoggerFactory.getLogger(this.getClass())

    @NonNull
    public SystemFile excelFileFromCoordinates(@NonNull @NotNull List<@Valid Coordinate> coordinateList) {
        try {
            File file = File.createTempFile(HEADER_EXCEL_FILE_PREFIX, HEADER_EXCEL_FILE_SUFIX)
            PoiSpreadsheetBuilder.create(file).build(w -> {
                w.apply(CoordinateExcelStylesheet.class)
                w.sheet(SHEET_NAME, s -> {
                    s.row(r -> {
                        [HEADER_NAME, HEADER_X, HEADER_Y, HEADER_Z].forEach {
                            r.cell(it)
                        }
                    })
                    coordinateList.stream()
                        .forEach( coordinate -> s.row(r -> {
                            r.cell(coordinate.getName());
                            r.cell(coordinate.getX());
                            r.cell(coordinate.getY());
                            r.cell(coordinate.getZ());
                        }));
                });
            });

            return new SystemFile(file).attach(HEADER_EXCEL_FILENAME);
        } catch (IOException e) {
            println("EXCEL ERROR: " + e)
            //LOG.error("File not found exception raised when generating excel file");
        }
        throw new HttpStatusException(HttpStatus.SERVICE_UNAVAILABLE, "error generating excel file")
    }
}
