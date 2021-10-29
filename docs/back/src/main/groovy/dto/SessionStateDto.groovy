package dto

import io.micronaut.core.annotation.Introspected
import ru.armbot.domain.Coordinate
import ru.armbot.domain.SessionState

@Introspected
class SessionStateDto {
    SessionState sessionState
    List<Coordinate> coordinateList
}
