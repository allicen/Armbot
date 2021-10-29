package ru.armbot.repository

import io.micronaut.data.annotation.Repository
import io.micronaut.data.repository.CrudRepository
import ru.armbot.domain.SessionState

@Repository
interface SessionStateRepository extends CrudRepository<SessionState, Long> {
    List<SessionState> list()
}