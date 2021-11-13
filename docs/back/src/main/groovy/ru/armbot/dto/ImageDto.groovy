package ru.armbot.dto

import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.Nullable

@Introspected
class ImageDto {

    String name
    String contentType
    byte[] imageByte
    int imagePositionX
    int imagePositionY
    boolean canEdit

    @Nullable
    Integer imageWidthPx
}
