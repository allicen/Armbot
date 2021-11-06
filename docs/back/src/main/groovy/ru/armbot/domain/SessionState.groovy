package ru.armbot.domain

import groovy.transform.CompileStatic
import groovy.transform.EqualsAndHashCode
import io.micronaut.core.annotation.Creator
import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.NonNull
import io.micronaut.core.annotation.Nullable

import javax.persistence.Entity
import javax.persistence.GeneratedValue
import javax.persistence.GenerationType
import javax.persistence.Id

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
    Integer imageId
    int imagePositionX
    int imagePositionY
    int imageSize
    boolean imageRequired = true

    @Creator
    SessionState(Integer imageId, int imagePositionX, int imagePositionY, int imageSize) {
        this.imageId = imageId
        this.imagePositionX = imagePositionX
        this.imagePositionY = imagePositionY
        this.imageSize = imageSize
    }

    SessionState() {}
}