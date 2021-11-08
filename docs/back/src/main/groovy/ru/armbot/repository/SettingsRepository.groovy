package ru.armbot.repository

import io.micronaut.data.annotation.Repository
import io.micronaut.data.repository.CrudRepository
import ru.armbot.domain.Settings

@Repository
interface SettingsRepository extends CrudRepository<Settings, Long>  {
    List<Settings> list()
}