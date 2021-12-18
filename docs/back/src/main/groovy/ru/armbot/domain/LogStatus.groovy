package ru.armbot.domain

/**
 * Статусы логов
 * */

enum LogStatus {
    ERROR('error', 'Ошибка'),
    INFO('info', 'Успех')

    private LogStatus(String id, String name) {
        this.id = id
        this.name = name
    }

    final String id
    final String name
}