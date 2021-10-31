import {Injectable} from "@angular/core";

@Injectable({ providedIn: 'root' })
export class SizeService {

    pxInOneMm: number = 3.7795751758;
    robotAreaMm: number = 640; // Рабочая зона робота - 640 мм в диаметре

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
}
