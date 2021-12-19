import { Injectable } from '@angular/core';
import {NgxRoslibService, Rosbridge, RosService} from 'ngx-roslib';
import {BehaviorSubject, Observable} from "rxjs";
import {Config} from "../config/config";

@Injectable({
  providedIn: 'root',
})
export class RosArmbotService {
    rbServer: Rosbridge;
    armbotIsConnected: boolean = false;

    private armbotStatus$: BehaviorSubject<string> = new BehaviorSubject<string>(this.config.robotStatus.disconnect);

    constructor(public roslibService: NgxRoslibService, private config: Config) {
        this.rbServer = this.roslibService.connect(config.webSocketRosUrl);
        if (!this.roslibService.onOpen || !this.roslibService.onClose || !this.roslibService.onError) {
            return;
        }
        this.roslibService.onOpen.subscribe(() => {
            this.armbotIsConnected = true;
            this.setArmbotStatus(this.config.robotStatus.ready);
            console.log('Connected to Rosbridge!')
        });
        this.roslibService.onClose.subscribe(() => {
            this.armbotIsConnected = false;
            this.setArmbotStatus(this.config.robotStatus.disconnect);
            console.log('Connection to Rosbridge closed');
        });
        this.roslibService.onError.subscribe(() => {
            this.armbotIsConnected = false;
            this.setArmbotStatus(this.config.robotStatus.disconnect);
            console.error('Error occurred with Rosbridge websocket')
        });
    }

    getArmbotStatus(): Observable<string> {
        return this.armbotStatus$.asObservable();
    }

    setArmbotStatus(value: string): void {
        this.armbotStatus$.next(value);
    }

    runArmbot(): void {
        if (!this.armbotIsConnected) {
           return;
        }

        this.setArmbotStatus(this.config.robotStatus.busy);
        const service = new RosService<{},{ topics: string[]; types: string[]; }>({
            ros: this.rbServer,
            name: '/set_position',
            serviceType: 'armbot_move/SetPosition',
        });

        service.call({position: 'TEST_123', x: 0.203389, y: -0.129121}, (msg) => {
            console.log(`ROS FINISH. RESULT`);
            this.setArmbotStatus(this.config.robotStatus.ready);
        });
    }
}
