package ru.armbot.repository

import io.micronaut.context.annotation.Executable
import io.micronaut.data.annotation.*
import io.micronaut.data.repository.CrudRepository
import ru.armbot.domain.Image

@Repository
interface ImageRepository extends CrudRepository<Image, Long> {
    Image find(String name)
    List<Image> list()
}