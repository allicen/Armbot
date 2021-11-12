package ru.armbot.utils

import javax.transaction.Transactional

@Transactional
class EnumService {
    // получить значение enum
    static def getValue (enumList, id) {
        for (item in enumList.values()) {
            if (item.id == id) {
                return item
            }
        }
        throw new IllegalArgumentException()
    }
}
