package ru.armbot.domain

import groovy.transform.CompileStatic
import groovy.transform.EqualsAndHashCode
import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.NonNull
import io.micronaut.core.annotation.Nullable

import javax.persistence.Entity
import javax.persistence.GeneratedValue
import javax.persistence.GenerationType
import javax.persistence.Id
import javax.validation.constraints.NotBlank

/**
 * Изображение
 * */

@Entity
@CompileStatic
@EqualsAndHashCode
@Introspected
class Image {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    Long id

    @Nullable
    String name

    @NonNull
    @NotBlank
    String contentType

    @NonNull
    @NotBlank
    byte[] imageByte

    Integer imagePositionX = 0

    Integer imagePositionY = 0

    @Nullable
    Integer imageWidthPx

    Image() { }

    Image(byte[] imageByte, String name, String contentType) {
        this.imageByte = imageByte
        this.name = name
        this.contentType = contentType
    }
}
