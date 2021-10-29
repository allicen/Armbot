package ru.armbot.repository

import io.micronaut.context.annotation.Executable
import io.micronaut.data.annotation.*
import io.micronaut.data.repository.CrudRepository
import ru.armbot.domain.Coordinate

@Repository
interface CoordinateRepository extends CrudRepository<Coordinate, Long> {
    @Executable Coordinate findByName(String name)
    List<Coordinate> list()
}