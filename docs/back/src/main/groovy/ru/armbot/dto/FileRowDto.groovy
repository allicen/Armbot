package ru.armbot.dto

class FileRowDto {
    // ID строки файла запуска
    long id

    // ID координаты
    long coordinateId

    // Задержка между нажатиями
    int delay = 0

    // Порядок сортировки строк
    int sortOrder = 0
}
