package ru.armbot.dto

import io.micronaut.core.annotation.Introspected

@Introspected
class FileRowDto {
    // ID строки файла запуска
    long id

    // ID координаты
    long coordinateId

    // Задержка между нажатиями
    double delay = 0

    // Порядок сортировки строк
    int sortOrder = 0
}
