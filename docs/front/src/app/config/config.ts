import {Injectable} from "@angular/core";

@Injectable({ providedIn: 'root' })
export class Config {
  httpUrl: string = 'http://localhost:9080';
  webSocketUrl: string = 'ws://localhost:9080/ws/coordinate';
  allowImageMimeTypes = ['image/jpeg', 'image/gif', 'image/png', 'image/svg+xml', 'image/tiff'];
  imageMaxSize: number = 104857600; // 100Mb
}
