package dto

import io.micronaut.core.annotation.Creator
import io.micronaut.core.annotation.Introspected
import io.micronaut.core.annotation.NonNull

import javax.validation.constraints.NotBlank
import javax.validation.constraints.NotNull

@Introspected
class ResponseDto {

    @NotNull
    ResponseStatus status

    String errorCode

    String message

    // Дополнительные опции
    def extendedResponse


    @Creator
    ResponseDto(@NonNull ResponseStatus status, String errorCode, String message, def extendedResponse) {
        this.status = status
        this.errorCode = errorCode
        this.message = message
        this.extendedResponse = extendedResponse
    }

    ResponseDto() {

    }

}
