package ru.armbot.dto

import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.Nullable
import ru.armbot.domain.Coordinate
import ru.armbot.domain.Image
import ru.armbot.domain.LaunchFileRow
import ru.armbot.domain.Settings

import javax.persistence.FetchType
import javax.persistence.OneToOne

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

    @Nullable
    @OneToOne(optional=false, fetch = FetchType.EAGER)
    List<FileRowDto> launchFileRowList
}