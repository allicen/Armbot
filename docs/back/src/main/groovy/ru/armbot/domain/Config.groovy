package ru.armbot.domain

import groovy.transform.CompileStatic
import groovy.transform.EqualsAndHashCode
import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.NonNull

import javax.persistence.Entity
import javax.persistence.GeneratedValue
import javax.persistence.GenerationType
import javax.persistence.Id
import javax.validation.constraints.NotBlank

/**
 * Конфиги робота
 * */

@Entity
@CompileStatic
@EqualsAndHashCode
@Introspected
class Config {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    Long id

    @NonNull
    @NotBlank
    String key

    @NonNull
    @NotBlank
    String value
}
