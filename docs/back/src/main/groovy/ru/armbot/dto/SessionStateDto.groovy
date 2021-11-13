package ru.armbot.dto

import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.Nullable
import ru.armbot.domain.Settings

import javax.persistence.FetchType
import javax.persistence.OneToOne

@Introspected
class SessionStateDto {
    @Nullable
    Long sessionId

    String workOption

    @Nullable
    ImageDto image

    @Nullable
    Settings settings

    List<CoordinateDto> coordinateList

    @Nullable
    @OneToOne(optional=false, fetch = FetchType.EAGER)
    List<FileRowDto> launchFileRowList
}