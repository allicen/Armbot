package ru.armbot.domain

import groovy.transform.CompileStatic
import groovy.transform.EqualsAndHashCode
import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.Nullable

import javax.persistence.Entity
import javax.persistence.FetchType
import javax.persistence.GeneratedValue
import javax.persistence.GenerationType
import javax.persistence.Id
import javax.persistence.ManyToOne

/**
 * Настройки пользовательского интерфейса
 * */

@Entity
@CompileStatic
@EqualsAndHashCode
@Introspected
class Settings {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    Long id

    // Радиус вокруг точки
    @Nullable
    Integer cursorArea

    @ManyToOne(optional=false, fetch = FetchType.EAGER)
    SessionState sessionState

    Settings() { }
}
