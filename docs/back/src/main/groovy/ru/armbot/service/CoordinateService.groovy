package ru.armbot.service

import jakarta.inject.Inject
import ru.armbot.repository.CoordinateRepository

class CoordinateService {
    private final String PREFIX = 'coordinate'

    @Inject CoordinateRepository coordinateRepository

    boolean nameExist(name) {
        return coordinateRepository.list().find{it.name == name }
    }

    String generateName() {
        int nameTail = coordinateRepository.list().size()
        String name = "${PREFIX}-${nameTail}"

        while (nameExist(name)) {
            nameTail++
            name = "${PREFIX}-${nameTail}"
        }

        return name
    }
}
