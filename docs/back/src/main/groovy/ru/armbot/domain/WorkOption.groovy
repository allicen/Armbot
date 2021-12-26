package ru.armbot.domain

/**
 * Режим работы
 * */

enum WorkOption {
    UPLOAD_IMAGE('uploadImage', 'Загрузить изображение клавиатуры'),
    UPLOAD_NO_IMAGE('uploadNoImage', 'Продолжить без загрузки клавиатуры'),
    UPLOAD_SESSION('uploadSession', 'Загрузить сессию'),
    UPLOAD_COORDINATE_LIST('uploadCoordinateList', 'Загрузить команды'),

    private WorkOption(String id, String name) {
        this.id = id
        this.name = name
    }

    final String id
    final String name
}