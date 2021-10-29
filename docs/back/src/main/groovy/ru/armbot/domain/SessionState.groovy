package ru.armbot.domain

import groovy.transform.CompileStatic
import groovy.transform.EqualsAndHashCode
import io.micronaut.core.annotation.Creator
import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.NonNull

import javax.persistence.Entity
import javax.persistence.FetchType
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

    int imageId
    int imagePositionX
    int imagePositionY
    int imageSize

    @Creator
    SessionState(int imageId, int imagePositionX, int imagePositionY, int imageSize) {
        this.imageId = imageId
        this.imagePositionX = imagePositionX
        this.imagePositionY = imagePositionY
        this.imageSize = imageSize
    }

    SessionState() {}
}