package ru.armbot.domain

import groovy.transform.CompileStatic
import groovy.transform.EqualsAndHashCode
import io.micronaut.core.annotation.Creator
import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.NonNull

import javax.persistence.Entity
import javax.persistence.GeneratedValue
import javax.persistence.GenerationType
import javax.persistence.Id
import javax.validation.constraints.NotBlank

@Entity
@CompileStatic
@EqualsAndHashCode
@Introspected
class Image {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    Long id

    final String name

    @NonNull
    @NotBlank
    final String contentType

    @NonNull
    @NotBlank
    final byte[] imageByte

    @Creator
    Image(@NonNull @NotBlank byte[] imageByte, @NonNull @NotBlank String contentType, String name) {
        this.name = name
        this.imageByte = imageByte
        this.contentType = contentType
    }

    Image() {

    }
}
