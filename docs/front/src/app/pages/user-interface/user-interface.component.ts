import {Component, ElementRef, Injectable, OnInit, ViewChild} from '@angular/core';
import { FileHandle } from './dragDrop.directive';
import {DomSanitizer} from "@angular/platform-browser";
import {HttpService} from "../../serviсes/http.service";
import { CdkDragEnd } from "@angular/cdk/drag-drop";
import {Coordinate, Position} from "../../model/models";
import {MatTable} from "@angular/material/table";
import {MatDialog} from "@angular/material/dialog";
import {OpenDialogComponent} from "./open-dialog/open-dialog.component";
import {StorageService} from "../../serviсes/storage.service";
import {MatSnackBar} from "@angular/material/snack-bar";
import {WebsocketService} from "../../serviсes/websocket.service";
import {Subject} from "rxjs";
import {Message} from "@angular/compiler/src/i18n/i18n_ast";
import {Config} from "../../config/config";
import {map} from "rxjs/operators";
import {SizeService} from "../../serviсes/size.service";

@Component({
  selector: 'app-root',
  templateUrl: './user-interface.component.html',
  styleUrls: ['./user-interface.component.less']
})
export class UserInterfaceComponent implements OnInit {
  title = 'docs';
  files: FileHandle[] = [];
  fileUpload: boolean = false;
  image: any = null;
  imageId: number | undefined;
  message: string | undefined;
  imageWidthPx: number | undefined;
  imageWidthMm: number | undefined;

  maxWidthLen: number = 4; // Макс количество символов для задания ширины
  dragImagePosition: Position = {x: 0, y: 0};
  editingAllowed: boolean = true;
  clickCoordinate: Coordinate | undefined;
  selectedPointIndex: number | undefined;
  exportCoordinateUrl: string = '';
  exportTxtCoordinateUrl: string = '';

  displayedColumns: string[] = ['name', 'x', 'y', 'z', 'action'];
  dataSource: Coordinate[] = [];

  gridStepPx: number = 100;
  gridStepMm: number = 10;

  gridLineThin: number = 1;
  gridVerticalCount: number = 0;
  gridHorizontalCount: number = 0;
  gridOn: boolean = false;
  gridColorDefault: string = '#cecece';
  gridColorValue: string = 'default';
  gridColors: string[] = ['default', 'black', 'yellow', 'red', 'green'];

  coordValidateMessage: string = '';
  validateError: boolean = true;

  robotAreaWidth: number = 0;
  gridHorizontalOffset: number = 200;

  @ViewChild(MatTable) table: MatTable<Coordinate> | undefined;
  @ViewChild("inputFile") inputFile: ElementRef | undefined;
  @ViewChild("uploadImage") uploadImage: ElementRef | undefined;
  @ViewChild("robotArea") robotArea: ElementRef | undefined;
  @ViewChild("exportExcel") exportExcel: ElementRef | undefined;
  @ViewChild("exportTxt") exportTxt: ElementRef | undefined;

  public messages: Subject<any> | undefined;

  constructor(private httpService: HttpService,
              private sanitizer: DomSanitizer,
              private dialog: MatDialog,
              private storage: StorageService,
              private _snackBar: MatSnackBar,
              private wsService: WebsocketService,
              private config: Config,
              private sizeService: SizeService) {

    this.wsService.connect(this.config.webSocketUrl).pipe().subscribe(res => {
        const response = JSON.parse(res.data);
        if (response.details) {
          const coordinate: Coordinate = response.details;
          this.dataSource.push(coordinate);
          if (this.table) {
            this.table.renderRows();
          }

          this.validateError = response.status === 'ERROR';
          this.coordValidateMessage = response.message ? response.message : '';
          this.hideMessage();
        }
    });
  }

  ngOnInit(): void {
    this.getSession();
    this.getImage();

    this.storage.getCoordinateDelete().subscribe(id => {

      const index = this.dataSource.findIndex(c => c.id == id);
      this.dataSource.splice(index, 1);
      if (this.table) {
        this.table.renderRows();
      }

      this.hideMessage();
    });

    this.storage.getCoordinateDeleteError().subscribe(err => this.validateError = err);
    this.storage.getCoordinateDeleteMessage().subscribe(mess => this.coordValidateMessage = mess);

    this.exportCoordinateUrl = this.httpService.getUrlExport();
    this.exportTxtCoordinateUrl = this.httpService.getUrlExportTxt();
  }

  ngAfterViewChecked() {
    if (this.robotArea) {
      this.robotAreaWidth = this.robotArea?.nativeElement.clientWidth;
      this.gridStepPx = this.sizeService.toPxTranslate(this.gridStepMm, 'mm', this.robotAreaWidth);
    }
  }

  filesDropped(files: FileHandle[]): void {
    if (files.length > 0) {
      // поддерживается загрузка несколльких файлов
      // загружаем только 1й файл!
      this.files = files.slice(0, 1);
      this.fileUpload = true;
    }
  }

  uploadFile(): any {
    if (this.files.length === 0) {
      return;
    }

    this.httpService.uploadImage(this.files[0]).pipe().subscribe((data => {
      if (data.status === 'OK') {
        this.getImage()
      }
      this.message = data.message;
    }));
  }

  removeFile() {
    this.files = [];
    this.fileUpload = false;
  }

  openDialogChangeFile($event: Event) {
    if (this.inputFile) {
      this.inputFile.nativeElement.click();
    }
  }

  changeFile($event: Event) {
    const element = $event.currentTarget as HTMLInputElement;
    let fileList: FileList | null = element.files;

    if (fileList) {
      const file = fileList[0];
      const url = this.sanitizer.bypassSecurityTrustUrl(window.URL.createObjectURL(file));
      this.files.push({ file, url });
    }

    if (this.files.length > 0) {
      this.fileUpload = true;
    }
  }

  getImage() {
    return this.httpService.getImage().subscribe((data: any) => {
      if (data.status === 'OK') {
        this.imageId = data.image.id;
        this.image = this.sanitizer.bypassSecurityTrustResourceUrl(`data:${data.image.contentType};base64,${data.image.imageByte}`);
        setTimeout(() => {
          this.getImageWidth();
          this.getGridCount();
        }, 1000);
      } else if (data.status === 'ERROR') {
        this.message = data.message;
      }
    });
  }

  getSession() {
    return this.httpService.getSession().subscribe((data: any) => {
      if (data.status === 'SUCCESS' && data.details) {
        const details = data.details.sessionState;
        this.imageWidthPx = details.imageSize;
        this.dragImagePosition = {x: details.imagePositionX, y: details.imagePositionY};
        this.editingAllowed = false;

        if (data.details.coordinateList) {
          this.dataSource = data.details.coordinateList;
        }
      }
    });
  }

  removeImage() {
    this.httpService.removeImage().subscribe((data: any) => {
      if (data.status === 'OK') {
        this.image = null;
        this.fileUpload = false;
        this.editingAllowed = true;
      }
      this.message = data.message;
      this.dataSource = [];
      this.dragImagePosition = {x: 0, y: 0};
      this.imageWidthMm = 0;
      this.imageWidthPx = 0;
    });

    this.files = [];
  }

  getImageWidth(): void {
      if (this.uploadImage) {
        this.imageWidthPx = this.uploadImage.nativeElement.width;

          if (this.imageWidthPx) {
              this.imageWidthMm = this.sizeService.fromPxTranslate(this.imageWidthPx, 'mm', this.robotAreaWidth);
          }
      }
  }

  renderImage(width: string) {
    if (width.length > this.maxWidthLen) {
      width = width.slice(0, this.maxWidthLen);
    }

    this.imageWidthPx = this.sizeService.toPxTranslate(Number(width), 'mm', this.robotAreaWidth);
  }


  getDragImagePosition($event: CdkDragEnd) {
    const position = $event.source.getFreeDragPosition();
    this.dragImagePosition.x = position.x;
    this.dragImagePosition.y = position.y;
  }

  setEditingCompleted() {
    this.editingAllowed = false;
    this.httpService.saveSessionState(this.imageId, this.imageWidthPx, this.dragImagePosition).subscribe(res => {

    });
  }

  setEditAllowed() {
    this.editingAllowed = true;
  }

  saveCoordinate($event: MouseEvent) {
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

  removeCoordinate(id: number) {
    this.dialog.open(OpenDialogComponent, {
      data: { id: id }
    });
  }

  changeCoordinateRow(value: any, type: string, id: number) {

    const coordinate = this.dataSource.filter(c => c.id === id).shift();

    if (!coordinate) {
      return;
    }

    switch (type) {
      case 'x':
        coordinate.x = value;
        break;
      case 'y':
        coordinate.y = value;
        break;
      case 'z':
        coordinate.z = value;
        break;
      case 'name':
        coordinate.name = value;
        break;
    }

    this.httpService.updateCoordinate(coordinate).pipe().subscribe((res) => {
      if (!res) {
        return;
      }

      this.validateError = res.status === 'ERROR';
      this.coordValidateMessage = res.message ? res.message : '';

      if (!this.validateError) {
        const index = this.dataSource.findIndex(c => c.id === coordinate.id);
        this.dataSource[index] = coordinate;
      }

      this.hideMessage();

      if (this.table) {
        this.table.renderRows();
      }
    });
  }

  exportFile() {
    this.exportExcel?.nativeElement.click();
  }

  coordinateSaveServer(coordinate: Coordinate) {
    this.httpService.saveCoordinate(coordinate).pipe().subscribe((res) => {

      if (!res) {
        return;
      }

      this.dataSource.push(res.details.coordinate);


      if (this.table) {
        this.table.renderRows();
      }

      this._snackBar.open(`Точка сохранена с координатами x=${coordinate.x}, y=${coordinate.y}`, 'X', {
        duration: 2000
      });
    });
  }

  setVisibleGrid(completed: boolean) {
    this.gridOn = completed;
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

  removeAllPoint() {
    this.httpService.removeAllCoordinates().pipe().subscribe((res) => {
      if (res.status === 'SUCCESS') {
        this.dataSource = [];
        this.clickCoordinate = undefined;
        this.selectedPointIndex = undefined;
      }
    });
  }

  showClickPoint(point: Coordinate, index: number) {
    this.clickCoordinate = point;
    this.selectedPointIndex = index;
  }

  exportFileTxt() {
    this.exportTxt?.nativeElement.click();
  }

  hideMessage() {
    if (!this.validateError) {
      setTimeout(() => {
        this.coordValidateMessage = '';
      }, 3000);
    }
  }

  getPosition(point: Position) {
    return this.sizeService.pointCoordinateToPx(point.x, point.y);
  }

}
