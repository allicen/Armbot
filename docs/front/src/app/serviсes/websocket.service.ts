import { Injectable } from "@angular/core";
import {Subject, Observable, Observer} from "rxjs";
import {Config} from "../config/config";

@Injectable()
export class WebsocketService {
    url: string = '';

    constructor(private config: Config) {
        this.url = this.config.webSocketUrl;
    }

    private subject: Subject<MessageEvent> | undefined;

    public connect(): Subject<MessageEvent> {
        if (!this.subject) {
            this.subject = this.create(this.url);
            console.log("Successfully connected: " + this.url);
        }
        return this.subject;
    }

    private create(url: string): Subject<MessageEvent> {
        let ws = new WebSocket(url);

        let observable = Observable.create((obs: Observer<MessageEvent>) => {
          ws.onmessage = obs.next.bind(obs);
          ws.onerror = obs.error.bind(obs);
          ws.onclose = obs.complete.bind(obs);
          return ws.close.bind(ws);
        });
        let observer = {
            next: (data: Object) => {
                if (ws.readyState === WebSocket.OPEN) {
                    ws.send(JSON.stringify(data));
                }
            }
        };
        return Subject.create(observer, observable);
    }
}
