package ru.armbot.service

import jakarta.inject.Inject
import ru.armbot.repository.CoordinateRepository
import ru.armbot.repository.ImageRepository
import ru.armbot.repository.LaunchFileRowRepository
import ru.armbot.repository.SessionStateRepository
import ru.armbot.repository.SettingsRepository

class SessionService {

    @Inject SessionStateRepository sessionStateRepository
    @Inject CoordinateRepository coordinateRepository
    @Inject ImageRepository imageRepository
    @Inject SettingsRepository settingsRepository
    @Inject LaunchFileRowRepository launchFileRowRepository

    def clearAll() {
        imageRepository.deleteAll()
        launchFileRowRepository.deleteAll()
        coordinateRepository.deleteAll()
        settingsRepository.deleteAll()
        sessionStateRepository.deleteAll()
    }
}
