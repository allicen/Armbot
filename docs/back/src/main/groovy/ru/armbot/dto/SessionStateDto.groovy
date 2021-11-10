package ru.armbot.dto

import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.Nullable
import ru.armbot.domain.Coordinate
import ru.armbot.domain.Image
import ru.armbot.domain.Settings

@Introspected
class SessionStateDto {
    @Nullable
    Long sessionId

    String workOption

    @Nullable
    Image image

    @Nullable
    Settings settings

    List<Coordinate> coordinateList
}