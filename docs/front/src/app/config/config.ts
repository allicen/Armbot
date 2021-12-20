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
}
