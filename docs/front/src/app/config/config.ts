import {Injectable} from "@angular/core";
import {Coordinate, WorkOption} from "../model/models";

@Injectable({ providedIn: 'root' })
export class Config {
  httpUrl: string = 'http://localhost:9080';
  webSocketUrl: string = 'ws://localhost:9080/ws/coordinate';
  webSocketRosUrl: string = 'ws://localhost:9090'; // ROS
  allowImageMimeTypes = ['image/jpeg', 'image/gif', 'image/png', 'image/svg+xml', 'image/tiff'];
  imageMaxSize: number = 104857600; // 100Mb
  workOptions: WorkOption[] = [
    {key: 'uploadImage', value: 'Загрузить изображение клавиатуры'},
    {key: 'uploadSession', value: 'Загрузить сеанс'},
    {key: 'uploadCoordinateList', value: 'Загрузить команды'}
  ];
  coordinateDefault: Coordinate = {x: 0, y: 0, z: 0, name: '', id: -1};
  robotStatus: any = {
    disconnect: 'disconnect', // Робот отключен
    busy: 'busy', // Робот занят
    ready: 'ready' // Робот готов
  };
  robotConfig: {key: string, value: null|string|number, name: string, type: string}[] = [
    {key: 'zPositionDefault', value: null, name: 'Z (высота) по-умолчанию', type: 'number'},
    {key: 'zPositionDefaultDown', value: null, name: 'Z (высота) по-умолчанию при нажатии кнопки', type: 'number'},
    {key: 'Orientation.x', value: null, name: 'Ориентация по X по умолчанию', type: 'number'},
    {key: 'Orientation.y', value: null, name: 'Ориентация по Y по умолчанию', type: 'number'},
    {key: 'Orientation.z', value: null, name: 'Ориентация по Z по умолчанию', type: 'number'},
    {key: 'Orientation.w', value: null, name: 'Ориентация по W по умолчанию', type: 'number'},
    {key: 'DefaultPosition.x', value: null, name: 'Позиция X по умолчанию', type: 'number'},
    {key: 'DefaultPosition.y', value: null, name: 'Позиция Y по умолчанию', type: 'number'},
    {key: 'DefaultPosition.z', value: null, name: 'Позиция Z по умолчанию', type: 'number'},
    {key: 'saveWebSocket', value: null, name: 'Сохранять координаты по вебсокету', type: 'boolean'},
    {key: 'saveToFile', value: null, name: 'Сохранять координаты в файл в проекте', type: 'boolean'}
  ];
}
