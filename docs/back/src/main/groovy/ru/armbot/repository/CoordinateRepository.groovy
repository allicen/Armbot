package ru.armbot.repository

import io.micronaut.context.annotation.Executable
import io.micronaut.data.annotation.Repository
import io.micronaut.data.repository.CrudRepository
import ru.armbot.domain.Coordinate

@Repository
interface CoordinateRepository extends CrudRepository<Coordinate, Long> {
    @Executable
    Coordinate find(String name)
    List<Coordinate> list()
}