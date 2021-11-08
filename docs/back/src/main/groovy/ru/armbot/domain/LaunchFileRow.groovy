package ru.armbot.domain

import groovy.transform.CompileStatic
import groovy.transform.EqualsAndHashCode
import io.micronaut.core.annotation.Introspected
import io.micronaut.core.value.MapPropertyResolver
import io.micronaut.data.jdbc.annotation.JoinColumn
import io.micronaut.data.jdbc.annotation.JoinTable
import org.hibernate.annotations.CascadeType

import javax.persistence.Entity
import javax.persistence.FetchType
import javax.persistence.GeneratedValue
import javax.persistence.GenerationType
import javax.persistence.Id
import javax.persistence.ManyToMany
import javax.persistence.ManyToOne
import javax.persistence.OneToMany
import javax.persistence.OneToOne

import static org.hibernate.annotations.CascadeType.*
import static org.hibernate.annotations.CascadeType.ALL as ALL

/**
 * Файл запуска
 * */

@Entity
@CompileStatic
@EqualsAndHashCode
@Introspected
class LaunchFileRow {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    Long id

    @OneToOne(optional=false, fetch = FetchType.LAZY)
    Coordinate coordinate

    int delay = 0

    int sortOrder = 0

    LaunchFileRow() { }
}