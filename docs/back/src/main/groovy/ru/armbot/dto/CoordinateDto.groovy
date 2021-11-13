package ru.armbot.dto

import io.micronaut.core.annotation.Introspected
import ru.armbot.domain.SizeUnit

@Introspected
class CoordinateDto {
    Long id
    String name
    double x
    double y
    double z
    SizeUnit unit
}
