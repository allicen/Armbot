import {Component, ElementRef, Input, OnInit, Output, ViewChild} from '@angular/core';
import {Coordinate, LaunchFileRow, Position} from "../../../../model/models";
import {SizeService} from "../../../../serviсes/size.service";
import {HttpService} from "../../../../serviсes/http.service";
import {MatSnackBar} from "@angular/material/snack-bar";
import {CdkDragEnd} from "@angular/cdk/drag-drop";
import {SessionService} from "../../../../serviсes/session.service";
import {StorageService} from "../../../../serviсes/storage.service";
import { UntilDestroy, untilDestroyed } from '@ngneat/until-destroy';
import {Config} from "../../../../config/config";

@UntilDestroy()
@Component({
  selector: 'app-robot-area',
  templateUrl: './robot-area.component.html',
  styleUrls: ['./robot-area.component.less']
})
export class RobotAreaComponent implements OnInit {

    dataSource: Coordinate[] = [];

    robotAreaWidth: number = 0;
    gridHorizontalOffset: number = 200;
    gridStepPx: number = 100;
    gridStepMm: number = 10;
    dragImagePosition: Position = {x: 0, y: 0};
    @Input() editingAllowed: boolean = true;
    imageWidthPx: number | undefined;
    imageWidthMm: number | undefined;
    maxWidthLen: number = 4; // Макс количество символов для задания ширины

    image: any = null;
    sessionExists: boolean = false;

    gridLineThin: number = 1;
    gridVerticalCount: number = 0;
    gridHorizontalCount: number = 0;
    gridOn: boolean = false;
    gridColorDefault: string = '#cecece';
    gridColorValue: string = 'default';
    gridColors: string[] = ['default', 'black', 'yellow', 'red', 'green'];

    clickCoordinate: Coordinate | undefined;
    autoGenerateFileOn: boolean = false;
    timerLastValue: number = 0;
    maxId: number = 0;
    diameterPoint: number = 6;

    createNewCommand: boolean = true;
    robotAreaHide: boolean = false;

    @Input() workOptionChecked: string = '';

    @ViewChild("uploadImage") uploadImage: ElementRef | undefined;
    @ViewChild("robotArea") robotArea: ElementRef | undefined;

    constructor(private sizeService: SizeService,
                private httpService: HttpService,
                private snackBar: MatSnackBar,
                private sessionService: SessionService,
                private storage: StorageService,
                private config: Config) {}

    ngOnInit(): void {

        // Работа без загрузки изображения
        if (this.workOptionChecked === 'uploadNoImage') {
            this.sessionService.setImageEditAllowed(false);
        }

        this.sessionService.getImagePosition().pipe(untilDestroyed(this)).subscribe(data => this.dragImagePosition = data);

        this.sessionService.getCoordinateList().pipe(untilDestroyed(this)).subscribe(data => {
            this.dataSource = data;
        });

        this.storage.getClickCoordinate().pipe(untilDestroyed(this)).subscribe(data => {
            this.clickCoordinate = data;
        });

        this.sessionService.getImageEditAllowed().pipe(untilDestroyed(this)).subscribe(data => {
            this.editingAllowed = data;
        });

        this.sessionService.getImageWidth().pipe(untilDestroyed(this)).subscribe(data => {
            if (data && data > 0) {
                this.imageWidthPx = data;
            }
        });

        this.sessionService.getImage().pipe(untilDestroyed(this)).subscribe(image => {
            if (image) {
                this.image = image;
                setTimeout(() => {
                    this.getImageWidth();
                    this.getGridCount();
                }, 1000);
            }
        });

        this.sessionService.getNextFileRowId().pipe(untilDestroyed(this)).subscribe(data => this.maxId = data);
        this.sessionService.getDiameterPoint().pipe(untilDestroyed(this)).subscribe(data => this.diameterPoint = data);
    }

    ngAfterViewChecked() {
        if (this.robotArea) {
            this.robotAreaWidth = this.robotArea?.nativeElement.clientWidth;
            this.gridStepPx = this.sizeService.toPxTranslate(this.gridStepMm, 'mm', this.robotAreaWidth);
        }
    }


    getPosition(point: Position) {
        return this.sizeService.pointCoordinateToPx(point.x, point.y);
    }

    saveCoordinate($event: MouseEvent) {

        // При клике по существующей точке новую не создает
        if (!this.createNewCommand) {
            this.createNewCommand = true;
            return;
        }

        // в режиме редактирования координаты не записываем
        if (this.editingAllowed) {
            return;
        }

        const image = this.uploadImage?.nativeElement;
        const robotAreaElem = this.robotArea?.nativeElement;

        let xPosition = 0;
        let yPosition = 0;

        let xScrollPos = image.scrollLeft || document.documentElement.scrollLeft;
        let yScrollPos = image.scrollTop || document.documentElement.scrollTop;

        xPosition += Math.round($event.x - (robotAreaElem.offsetLeft - xScrollPos + image.clientLeft));
        yPosition += Math.round($event.y - (robotAreaElem.offsetTop - yScrollPos + image.clientTop));

        const coordinateMm: Position = this.sizeService.pointCoordinateToMm(xPosition, yPosition);

        const coordinate: Coordinate = {id: 0, name: '', x: coordinateMm.x, y: coordinateMm.y, z: 0};

        this.coordinateSaveServer(coordinate);
    }


    coordinateSaveServer(coordinate: Coordinate) {

        this.httpService.saveCoordinate(coordinate).pipe(untilDestroyed(this)).subscribe((res) => {

            if (!res) {
                return;
            }

            this.sessionService.addCoordinateInList(res.details.coordinate);
            this.saveLaunchFileRow(res.details.coordinate);

            this.snackBar.open(`Команда сохранена с координатами x=${coordinate.x}, y=${coordinate.y}`, 'X', {
                duration: 2000
            });
        });
    }


    getDragImagePosition($event: CdkDragEnd) {
        const position = $event.source.getFreeDragPosition();
        this.sessionService.setImagePosition(position.x, position.y);
    }

    renderImage(width: string) {
        if (width.length > this.maxWidthLen) {
            width = width.slice(0, this.maxWidthLen);
        }

        this.imageWidthPx = this.sizeService.toPxTranslate(Number(width), 'mm', this.robotAreaWidth);
    }

    setVisibleGrid(completed: boolean) {
        this.gridOn = completed;
        this.getGridCount();
    }

    changeGreedStep(value: string) {
        this.gridStepMm = Number(value);
        this.gridStepPx = this.sizeService.toPxTranslate(Number(value), 'mm', this.robotAreaWidth);
        this.getGridCount();
    }

    getGridCount() {
        if (this.robotArea) {
            // сетку строим от центра к краям поля
            this.gridVerticalCount = Math.round((this.robotArea.nativeElement.clientWidth / 2) / (this.gridStepPx - this.gridLineThin / this.gridStepPx));
            this.gridHorizontalCount = Math.round(((this.robotArea.nativeElement.clientHeight + this.gridHorizontalOffset) / 2) / (this.gridStepPx - this.gridLineThin / this.gridStepPx));
        }
    }

    setEditingCompleted() {
        this.sessionService.setImageEditAllowed(false);
        this.httpService.setImageDetails(this.dragImagePosition, this.imageWidthPx || 0, false)
            .pipe(untilDestroyed(this)).subscribe();

        if (this.imageWidthPx) {
            this.sessionService.setImageWidth(this.imageWidthPx);
        }

        this.storage.setCurrentStep(3);
    }

    getImageWidth(): void {
        if (this.uploadImage) {
            this.imageWidthPx = this.uploadImage.nativeElement.width;

            if (this.imageWidthPx) {
                this.imageWidthMm = this.sizeService.fromPxTranslate(this.imageWidthPx, 'mm', this.robotAreaWidth);
            }
        }
    }

    autoGenerateFileStart() {
        this.autoGenerateFileOn = true;
    }

    autoGenerateFileStop() {
        this.autoGenerateFileOn = false;
        this.timerLastValue = 0;
        this.storage.setClickCoordinate(this.config.coordinateDefault);
    }

    changeRadiusPoint(value: string) {
      const size: number = Number(value);
      this.sessionService.setDiameterPoint(size);
      this.httpService.setCursorPoint(size).pipe(untilDestroyed(this)).subscribe(data => {
        if (!data) {
          return;
        }
        this.diameterPoint = size;
      });
    }

    copyCoordinate(clickCoordinate: Coordinate) {
        this.createNewCommand = false;
        this.saveLaunchFileRow(clickCoordinate);
        this.storage.setClickCoordinate(clickCoordinate);
    }

    saveLaunchFileRow(coordinate: Coordinate) {
        if (!this.autoGenerateFileOn) {
            return;
        }

        const now: number = Date.now();
        const timerClick = this.timerLastValue === 0 ? 0 : now;
        const timer: number = timerClick - this.timerLastValue;

        const next: number = this.maxId + 1;
        const launchFileRow: LaunchFileRow = {id: next, coordinate: coordinate, delay: timer/1000, sortOrder: next};
        this.sessionService.addLaunchFileRowList(launchFileRow);

        this.timerLastValue = now;
    }

    changeRobotAreaVis() {
        this.robotAreaHide = !this.robotAreaHide;
    }
}
