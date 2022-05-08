import { Injectable } from '@angular/core';
import {NgxRoslibService, Rosbridge, RosService} from 'ngx-roslib';
import {BehaviorSubject, Observable} from "rxjs";
import {Config} from "../config/config";
import {Coordinate} from "../model/models";

@Injectable({
  providedIn: 'root',
})
export class RosArmbotService {
    rbServer: Rosbridge;
    armbotIsConnected: boolean = false;

    private armbotStatus$: BehaviorSubject<string> = new BehaviorSubject<string>(this.config.robotStatus.disconnect);

    constructor(public roslibService: NgxRoslibService, private config: Config) {
        this.rbServer = this.roslibService.connect(this.config.webSocketRosUrl);
        this.armbotConnect();
    }

    armbotConnect(): void {
        this.rbServer = this.roslibService.connect(this.config.webSocketRosUrl);
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

    runArmbotCommand(coordinate: Coordinate): void {
        if (!this.armbotIsConnected) {
           return;
        }

        this.setArmbotStatus(this.config.robotStatus.busy);
        const service = new RosService<{},{ topics: string[]; types: string[]; }>({
            ros: this.rbServer,
            name: '/set_position',
            serviceType: 'armbot_move/SetPosition',
        });

        service.call({position: coordinate.name,
                          x: coordinate.x/1000, // в ROS расстояния в метрах
                          y: coordinate.y/1000,
                          z: coordinate.z/1000,
                          pressing: false}, (msg) => {
            console.log(`ROS FINISH. RESULT`);
            this.setArmbotStatus(this.config.robotStatus.ready);
        });
    }

    runArmbotLaunch(filesPath: any): void {
        this.setArmbotStatus(this.config.robotStatus.busy);
        const service = new RosService<{},{ topics: string[]; types: string[]; }>({
            ros: this.rbServer,
            name: '/armbot_run',
            serviceType: 'armbot_move/RunArmbot',
        });

        service.call({'command_description': filesPath.command_description || '', 'commands': filesPath.commands || ''}, (msg) => {
            console.log(`ROS LAUNCH FINISH. RESULT`);
            this.setArmbotStatus(this.config.robotStatus.ready);
        });
    }

    runMotor(motorNumber: number, stepCount: number, direction: number): void {
        this.setArmbotStatus(this.config.robotStatus.busy);
        const service = new RosService<{},{ topics: string[]; types: string[]; }>({
            ros: this.rbServer,
            name: '/motor_run',
            serviceType: 'armbot_move/RunMotor',
        });

        service.call({'motorNumber': motorNumber, 'stepCount': stepCount, 'direction': direction}, (msg) => {
            console.log(`ROS MOTOR FINISH. RESULT`);
            this.setArmbotStatus(this.config.robotStatus.ready);
        });
    }

    runMotorStart(): void {
        this.setArmbotStatus(this.config.robotStatus.busy);
        const service = new RosService<{},{ topics: string[]; types: string[]; }>({
            ros: this.rbServer,
            name: '/motor_run_start',
            serviceType: 'armbot_move/RunMotorStart',
        });

        service.call({}, (msg) => {
            console.log(`ROS MOTOR FINISH. RESULT`);
            this.setArmbotStatus(this.config.robotStatus.ready);
        });
    }

    /**
     * Вернуть модель робота в исходное положение
     * */
    returnDefaultPosition(): void  {
        const service = new RosService<{},{ topics: string[]; types: string[]; }>({
            ros: this.rbServer,
            name: '/return_default_position_model',
            serviceType: 'armbot_move/DefaultService',
        });
        service.call({}, (msg) => {
            console.log(`ROS MODEL FINISH. RESULT`);
            this.setArmbotStatus(this.config.robotStatus.ready);
        });
    }
}
