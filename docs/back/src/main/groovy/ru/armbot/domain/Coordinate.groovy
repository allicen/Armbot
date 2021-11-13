package ru.armbot.domain

import groovy.transform.CompileStatic
import groovy.transform.EqualsAndHashCode
import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.NonNull

import javax.persistence.Entity
import javax.persistence.FetchType
import javax.persistence.GeneratedValue
import javax.persistence.GenerationType
import javax.persistence.Id
import javax.persistence.ManyToOne
import javax.validation.constraints.NotBlank

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

    @ManyToOne(optional=false, fetch = FetchType.EAGER)
    SessionState sessionState

    Coordinate() {
    }
}
