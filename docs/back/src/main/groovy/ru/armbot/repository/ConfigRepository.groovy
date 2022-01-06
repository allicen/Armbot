package ru.armbot.repository

import io.micronaut.data.annotation.Repository
import io.micronaut.data.repository.CrudRepository
import ru.armbot.domain.Config

@Repository
interface ConfigRepository extends CrudRepository<Config, Long> {
    List<Config> list()
}