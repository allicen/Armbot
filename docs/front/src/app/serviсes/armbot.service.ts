import {Injectable} from "@angular/core";
import {BehaviorSubject, Observable} from "rxjs";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {RosArmbotService} from "./roslib.service";
import {Config} from "../config/config";
import {Coordinate} from "../model/models";

@UntilDestroy()
@Injectable({ providedIn: 'root' })
export class ArmbotService {

    private armbotMessage$: BehaviorSubject<string> = new BehaviorSubject<string>('');
    private armbotStatus$: BehaviorSubject<string> = new BehaviorSubject<string>('');
    private armbotButtonDisabled$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false);

    constructor(public rosArmbotService: RosArmbotService, private config: Config) {
        this.rosArmbotService.getArmbotStatus().pipe(untilDestroyed(this)).subscribe(status => {
            this.setArmbotStatus(status);
            this.setArmbotButtonDisabled(status === this.config.robotStatus.disconnect || status === this.config.robotStatus.busy);
            if (status === this.config.robotStatus.disconnect) {
                this.setArmbotMessage('Робот отключен');
            } else if (status === this.config.robotStatus.busy) {
                this.setArmbotMessage('Робот занят');
            } else if (status === this.config.robotStatus.ready) {
                this.setArmbotMessage('Робот свободен');
            }
        });
    }

    armbotOn(): void {
        this.rosArmbotService.armbotConnect();
    }

    getArmbotMessage(): Observable<string> {
        return this.armbotMessage$.asObservable();
    }

    setArmbotMessage(message: string): void {
        this.armbotMessage$.next(message);
    }

    getArmbotStatus(): Observable<string> {
      return this.armbotStatus$.asObservable();
    }

    setArmbotStatus(status: string): void {
      this.armbotStatus$.next(status);
    }

    getArmbotButtonDisabled(): Observable<boolean> {
      return this.armbotButtonDisabled$.asObservable();
    }

    setArmbotButtonDisabled(disabled: boolean): void {
      this.armbotButtonDisabled$.next(disabled);
    }

    runArmbotCommand(coordinate: Coordinate): void {
        this.rosArmbotService.runArmbotCommand(coordinate);
    }

    runArmbotLaunch(filesPath: any): void {
        this.rosArmbotService.runArmbotLaunch(filesPath);
    }

    runMotor(motorNumber: number, stepCount: number, direction: number): void {
        this.rosArmbotService.runMotor(motorNumber, stepCount, direction);
    }

    runMotorStart(): void {
        this.rosArmbotService.runMotorStart();
    }
}
