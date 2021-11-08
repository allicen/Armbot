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
import java.time.ZonedDateTime

/**
 * Координата
 * */

@Entity
@CompileStatic
@EqualsAndHashCode
@Introspected
class Coordinate {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    Long id

    @NonNull
    @NotBlank
    String name

    @NonNull
    @NotBlank
    double x

    @NonNull
    @NotBlank
    double y

    @NonNull
    @NotBlank
    double z = 0

    SizeUnit unit = SizeUnit.MM

    @Nullable
    ZonedDateTime timeCreate

    Coordinate() {
    }
}
