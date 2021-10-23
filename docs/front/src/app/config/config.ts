import {Injectable} from "@angular/core";

@Injectable({ providedIn: 'root' })
export class Config {
  httpUrl: string = 'http://0.0.0.0:9080';
}
