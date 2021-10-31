package dto

import io.micronaut.core.annotation.Creator
import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.NonNull
import ru.armbot.domain.ResponseStatus

import javax.validation.constraints.NotNull

@Introspected
class ResponseDto {

    @NotNull
    ResponseStatus status

    String errorCode

    String message

    // Дополнительные опции
    def details


    @Creator
    ResponseDto(@NonNull ResponseStatus status, String errorCode, String message, def details) {
        this.status = status
        this.errorCode = errorCode
        this.message = message
        this.details = details
    }

    ResponseDto() {

    }

}
