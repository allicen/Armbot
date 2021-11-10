package ru.armbot.domain

import groovy.transform.CompileStatic
import groovy.transform.EqualsAndHashCode
import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.NonNull
import io.micronaut.core.annotation.Nullable

import javax.persistence.Entity
import javax.persistence.FetchType
import javax.persistence.GeneratedValue
import javax.persistence.GenerationType
import javax.persistence.Id
import javax.persistence.OneToOne

/**
 * Сеанс пользователя
 * */

@Entity
@CompileStatic
@EqualsAndHashCode
@Introspected
class SessionState {
    @Id
    @NonNull
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    Long id

    @Nullable
    @OneToOne(optional=false, fetch = FetchType.EAGER)
    Image image

    @Nullable
    @OneToOne(optional=false, fetch = FetchType.EAGER)
    Settings settings

    @Nullable
    WorkOption workOption

    SessionState() {}
}