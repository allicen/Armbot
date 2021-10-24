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
class Coordinate {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    Long id

    @NonNull
    @NotBlank
    final String name

    @NonNull
    @NotBlank
    final int x

    @NonNull
    @NotBlank
    final int y

    @NonNull
    @NotBlank
    final int z

    @Creator
    Coordinate(@NonNull @NotBlank String name, @NonNull @NotBlank int x, @NonNull @NotBlank int y, @NonNull @NotBlank int z) {
        this.name = name
        this.x = x
        this.y = y
        this.z = z
    }

    Coordinate() {

    }
}
