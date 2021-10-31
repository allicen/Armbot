import {Injectable} from "@angular/core";
import {Position} from "../model/models";

@Injectable({ providedIn: 'root' })
export class SizeService {

    pxInOneMm: number = 3.7795751758;
    // Зона робота - квадрат
    robotAreaMm: number = 640; // Рабочая зона робота - 640 мм в диаметре
    robotAreaPx: number = 1200; // Рабочая зона в px (на картинке высота обрезана снизу на 200 px)

    constructor() {}

    fromPxTranslate(size: number, type: string, robotAreaPx: number): number {
        const offset = this.getOffset(robotAreaPx);
        let result = 0;

        switch (type) {
            case 'mm':
                result = size / this.pxInOneMm / offset;
                break;
            case 'sm':
                result = size / this.pxInOneMm / offset * 10;
                break;
            case 'm':
                result = size / this.pxInOneMm / offset * 100;
                break
            default:
                return 0;
        }

        return Math.round(result);
    }


    toPxTranslate(size: number, type: string, robotAreaPx: number): number {
        const offset = this.getOffset(robotAreaPx);
        let result = 0;

        switch (type) {
            case 'mm':
                result = size * this.pxInOneMm * offset;
                break;
            case 'sm':
                result = size * this.pxInOneMm * offset * 10;
                break;
            case 'm':
                result = size * this.pxInOneMm * offset * 100;
                break;
            default:
                return 0;
        }

        return Math.round(result);
    }


    getOffset(robotAreaPx: number): number {
        return  robotAreaPx / (this.robotAreaMm * this.pxInOneMm);
    }

    pointCoordinateToPx(x: number, y: number, robotAreaPx: number = this.robotAreaPx): Position {
        const xPx = this.toPxTranslate(x, 'mm', robotAreaPx);
        const yPx = this.toPxTranslate(y, 'mm', robotAreaPx);

        return {
            /// Меняем оси
            x: robotAreaPx / 2 - yPx,
            y: robotAreaPx / 2 - xPx
        };
    }

    pointCoordinateToMm(x: number, y: number, robotAreaPx = this.robotAreaPx): Position {
        const xMm = this.fromPxTranslate(x, 'mm', robotAreaPx);
        const yMm = this.fromPxTranslate(y, 'mm', robotAreaPx);

        return {
            /// Меняем оси
            x: this.robotAreaMm / 2 - yMm,
            y: this.robotAreaMm / 2 - xMm
        };

    }
}
