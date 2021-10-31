package ru.armbot.domain

enum SizeUnit {
    MM('mm', 'мм'),
    SM('sm', 'см'),
    M('m', 'м'),
    PX('px', 'пикселей')

    private SizeUnit(String id, String name) {
        this.id = id
        this.name = name
    }

    final String id
    final String name
}
