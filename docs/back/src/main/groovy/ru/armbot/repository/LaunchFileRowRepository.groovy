package ru.armbot.repository

import io.micronaut.data.annotation.Repository
import io.micronaut.data.repository.CrudRepository
import ru.armbot.domain.LaunchFileRow

@Repository
interface LaunchFileRowRepository extends CrudRepository<LaunchFileRow, Long> {
    List<LaunchFileRow> list()
}
