import {Injectable} from "@angular/core";
import {WorkOption} from "../model/models";

@Injectable({ providedIn: 'root' })
export class Config {
  httpUrl: string = 'http://localhost:9080';
  webSocketUrl: string = 'ws://localhost:9080/ws/coordinate';
  allowImageMimeTypes = ['image/jpeg', 'image/gif', 'image/png', 'image/svg+xml', 'image/tiff'];
  imageMaxSize: number = 104857600; // 100Mb
  workOption: WorkOption[] = [
    {key: 'uploadSession', value: 'Загрузить сессию'},
    {key: 'uploadImage', value: 'Загрузить изображение клавиатуры'},
    {key: 'uploadCoordinateList', value: 'Загрузить координаты'}
  ];
}
