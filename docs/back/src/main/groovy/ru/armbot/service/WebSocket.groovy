package ru.armbot.service

import ru.armbot.dto.ResponseDto
import ru.armbot.domain.ResponseStatus
import io.micronaut.core.annotation.Creator
import io.micronaut.websocket.WebSocketBroadcaster
import io.micronaut.websocket.WebSocketSession
import io.micronaut.websocket.annotation.OnClose
import io.micronaut.websocket.annotation.OnMessage
import io.micronaut.websocket.annotation.OnOpen
import io.micronaut.websocket.annotation.ServerWebSocket
import jakarta.inject.Inject
import org.reactivestreams.Publisher
import ru.armbot.domain.Coordinate
import ru.armbot.repository.CoordinateRepository
import ru.armbot.utils.SizeUnitUtils

@ServerWebSocket("/ws/coordinate")
class WebSocket {
    @Inject CoordinateRepository coordinateRepository
    @Inject CoordinateService coordinateService

    private WebSocketBroadcaster broadcaster

    @Creator
    WebSocket(WebSocketBroadcaster broadcaster) {
        this.broadcaster = broadcaster
    }

    @OnOpen
    Publisher<ResponseDto> onOpen() {
        return broadcaster.broadcast(new ResponseDto(status: ResponseStatus.CONNECT))
    }

    @OnMessage
    Publisher<ResponseDto> onMessage(String message, WebSocketSession session) {
        List<String> messArr = message.split(' ')
        if (messArr.size() != 3) {
            return broadcaster.broadcast(new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'INVALID_DATA',
                                                         message: 'Ошибка в данных координат'))
        }

        try {
            Coordinate coordinate = new Coordinate(name: coordinateService.generateName(),
                                                   x: SizeUnitUtils.fromM(messArr[0]),
                                                   y: SizeUnitUtils.fromM(messArr[1]),
                                                   z: SizeUnitUtils.fromM(messArr[2]))
            coordinateRepository.save(coordinate)
            return broadcaster.broadcast(new ResponseDto(status: ResponseStatus.SUCCESS,
                                                         message: 'Сохранена новая координата по вебсокету', details: coordinate))
        } catch (e) {
            println(e)
            return broadcaster.broadcast(new ResponseDto(status: ResponseStatus.ERROR, errorCode: 'ERROR_SAVE',
                    message: 'Ошибка сохранения координаты'))
        }
    }

    @OnClose
    Publisher<ResponseDto> onClose() {
        return broadcaster.broadcast(new ResponseDto(status: ResponseStatus.DISCONNECT))
    }
}
