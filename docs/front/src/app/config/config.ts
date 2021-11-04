import {Injectable} from "@angular/core";

@Injectable({ providedIn: 'root' })
export class Config {
  httpUrl: string = 'http://localhost:9080';
  webSocketUrl: string = 'ws://localhost:9080/ws/coordinate';
}
