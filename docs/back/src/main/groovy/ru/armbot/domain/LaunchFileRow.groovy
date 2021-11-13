package ru.armbot.domain

import groovy.transform.CompileStatic
import groovy.transform.EqualsAndHashCode
import io.micronaut.core.annotation.Introspected

import javax.persistence.Entity
import javax.persistence.FetchType
import javax.persistence.GeneratedValue
import javax.persistence.GenerationType
import javax.persistence.Id
import javax.persistence.ManyToOne

/**
 * Строка файла запуска
 * */

@Entity
@CompileStatic
@EqualsAndHashCode
@Introspected
class LaunchFileRow {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    Long id

    @ManyToOne(optional=false, fetch = FetchType.EAGER)
    Coordinate coordinate

    double delay = 0

    int sortOrder = 0

    @ManyToOne(optional=false, fetch = FetchType.EAGER)
    SessionState sessionState

    LaunchFileRow() { }
}
