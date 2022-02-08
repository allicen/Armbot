import {Injectable} from "@angular/core";
import {Coordinate, Motor, WorkOption} from "../model/models";

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
    {key: 'defaultOrientation_x', value: null, name: 'Ориентация по X по умолчанию', type: 'number'},
    {key: 'defaultOrientation_y', value: null, name: 'Ориентация по Y по умолчанию', type: 'number'},
    {key: 'defaultOrientation_z', value: null, name: 'Ориентация по Z по умолчанию', type: 'number'},
    {key: 'defaultOrientation_w', value: null, name: 'Ориентация по W по умолчанию', type: 'number'},
    {key: 'defaultPosition_x', value: null, name: 'Позиция X по умолчанию', type: 'number'},
    {key: 'defaultPosition_y', value: null, name: 'Позиция Y по умолчанию', type: 'number'},
    {key: 'defaultPosition_z', value: null, name: 'Позиция Z по умолчанию', type: 'number'},
    {key: 'saveWebSocket', value: null, name: 'Сохранять координаты по вебсокету', type: 'boolean'},
    {key: 'saveToFile', value: null, name: 'Сохранять координаты в файл в проекте', type: 'boolean'}
  ];
  motors: Motor[] = [
    {key: 1, value: 'Нижний двигатель', forwardDirection: 'Влево', inverseDirection: 'Вправо'},
    {key: 2, value: 'Левый двигатель', forwardDirection: 'Назад', inverseDirection: 'Вперед'},
    {key: 3, value: 'Правый двигатель', forwardDirection: 'Вниз', inverseDirection: 'Вверх'},
    {key: 4, value: 'Сервопривод - поворот рабочего инструмента', forwardDirection: 'Вперед', inverseDirection: 'Назад'},
    {key: 5, value: 'Сервопривод - размыкание захвата', forwardDirection: 'Вперед', inverseDirection: 'Назад'}
  ];
}
