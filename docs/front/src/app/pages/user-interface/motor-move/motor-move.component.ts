import {Component, OnInit} from '@angular/core';
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {Motor} from "../../../model/models";
import {Config} from "../../../config/config";
import {ArmbotService} from "../../../serviсes/armbot.service";

@UntilDestroy()
@Component({
  selector: 'app-motor-move',
  templateUrl: './motor-move.component.html',
  styleUrls: ['./motor-move.component.less']
})
export class MotorMoveComponent implements OnInit {

    constructor(private config: Config, private armbotService: ArmbotService) { }

    stepCountDefault: number = 46; //  количество шагов для двигателя
    degreesCountDefault: number = 10; // количество градусов для сервопривода

    currentMotor: number = 1;
    changeStepCount: boolean = true;
    stepCount: number = this.stepCountDefault;
    degreesCount: number = this.degreesCountDefault;
    stepDegreesValue: number = this.stepCountDefault;
    motors: Motor[] = [];
    armbotOnTurn: boolean = false;
    armbotButtonDisabled: boolean = true;

    ngOnInit(): void {
        this.motors = this.config.motors;

      this.armbotService.getArmbotStatus().pipe(untilDestroyed(this)).subscribe(data => {
         this.armbotOnTurn = data !== 'disconnect';
      });

      this.armbotService.getArmbotButtonDisabled().pipe(untilDestroyed(this)).subscribe(data => this.armbotButtonDisabled = data);
    }

    changeMotor(key: number): void {
        this.currentMotor = key;
        this.changeStepCount = key === 1 || key === 2 || key === 3;
        this.stepDegreesValue = this.changeStepCount ? this.stepCount : this.degreesCount;
    }

    // Направление: 0 - Вперед, 1 - Назад
    runMotor(direction: number): void {
        this.armbotService.runMotor(this.currentMotor, this.stepCount, direction);
    }

    changeSteps(steps: string): void {
        if (this.changeStepCount) {
          this.stepCount = Number(steps);
        } else {
          this.degreesCount = Number(steps);
        }
    }

    runMotorStart(): void {
       this.armbotService.runMotorStart();
    }
}
