import { Injectable } from '@angular/core';
import {NgxRoslibService, Rosbridge, RosoutMessage, RosService, RosTopic} from 'ngx-roslib';
import {BehaviorSubject, Observable} from "rxjs";
import {Config} from "../config/config";
import {CameraImage, Coordinate, RobotInfoFromCamera} from "../model/models";

@Injectable({
  providedIn: 'root',
})
export class RosArmbotService {
    rbServer: Rosbridge;
    armbotIsConnected: boolean = false;

    private armbotStatus$: BehaviorSubject<string> = new BehaviorSubject<string>(this.config.robotStatus.disconnect);
    private armbotCameraImage$: BehaviorSubject<CameraImage> = new BehaviorSubject<CameraImage>({
        data: null,
        encoding: null,
        width: 600,
        height: 300
    });
    private robotDiagnosticInfo$: BehaviorSubject<RobotInfoFromCamera> = new BehaviorSubject<RobotInfoFromCamera>({
        resultExists: null,
        info:  null,
        isStartPosition: null,
        xError: null,
        yError: null,
        zError: null,
        joint1: null,
        joint2: null,
        joint3: null,
        joint4: null,
    });

    private robotReturnDefaultPosition$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false);

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

    /**
     * Вернуться в исходное положение по камере
     * */
    returnDefaultPositionCamera() {
        const service = new RosService<{},{ topics: string[]; types: string[]; }>({
            ros: this.rbServer,
            name: '/return_default_pos_camera',
            serviceType: 'armbot_camera/DefaultService',
        });
        service.call({}, (msg) => {
            console.log(`ROS MODEL CAMERA FINISH. RESULT`);
            this.setRobotDiagnosticInfo({
                resultExists: null,
                info: null,
                isStartPosition: null,
                xError: null,
                yError: null,
                zError: null,
                joint1: null,
                joint2: null,
                joint3: null,
                joint4: null,
            });
            this.robotReturnDefaultPosition$.next(true);
        });
    }

    getRobotReturnDefaultPosition(): Observable<boolean> {
        return this.robotReturnDefaultPosition$.asObservable();
    }

    /**
     * Получить изображение с камеры
     * */
     getImageFromCamera(): Observable<any> {

         const rosout = new RosTopic<RosoutMessage>({
             ros: this.rbServer,
             name: '/room_camera_one',
             messageType: 'armbot_camera/ImageCamera',
         });
         rosout.subscribe((msg: any) => {
             if (msg) {
                 this.armbotCameraImage$.next({
                     data: msg.data,
                     encoding: msg.encoding,
                     width: msg.width,
                     height: msg.height
                 })
             }
         });

         return this.armbotCameraImage$.asObservable();
     }

     getRobotDiagnosticInfo():Observable<RobotInfoFromCamera> {
        return this.robotDiagnosticInfo$.asObservable();
     }

     setRobotDiagnosticInfo(msg: any) {
        this.robotDiagnosticInfo$.next({
          resultExists: msg.is_start_position || msg.x_error || msg.y_error || msg.z_error,
          info:  msg.info,
          isStartPosition: msg.is_start_position,
          xError: msg.x_error,
          yError: msg.y_error,
          zError: msg.z_error,
          joint1: msg.joint_1,
          joint2: msg.joint_2,
          joint3: msg.joint_3,
          joint4: msg.joint_4
        })
     }

    /**
     *  Диагностика робота (получить инфу об отклонениях от начальной позиции)
     * */
    robotDiagnostics() {
        const service = new RosService<{},{ topics: string[]; types: string[]; }>({
            ros: this.rbServer,
            name: '/robot_info_by_camera',
            serviceType: 'armbot_camera/RobotInfoService',
        });
        service.call({}, (msg) => {
            this.setRobotDiagnosticInfo(msg);
        });
    }
}
