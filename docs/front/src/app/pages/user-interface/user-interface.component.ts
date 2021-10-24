import {Component, ElementRef, Injectable, OnInit, ViewChild} from '@angular/core';
import { FileHandle } from './dragDrop.directive';
import {DomSanitizer} from "@angular/platform-browser";
import {HttpService} from "../../serviсes/http.service";
import { CdkDragEnd } from "@angular/cdk/drag-drop";
import {Coordinate} from "../../model/models";
import {MatTable} from "@angular/material/table";
import {MatDialog} from "@angular/material/dialog";
import {OpenDialogComponent} from "./open-dialog/open-dialog.component";
import {StorageService} from "../../serviсes/storage.service";
import {config} from "rxjs";

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
  message: string | undefined;
  imageWidth: number = 0;
  maxWidthLen: number = 4; // Макс количество символов для задания ширины
  dragImagePosition = {x: 0, y: 0};
  editingAllowed: boolean = true;
  pointNameDefault: string = 'coordinate'
  coordinateSaved: boolean = false;
  exportCoordinateUrl: string = '';

  displayedColumns: string[] = ['name', 'x', 'y', 'z', 'action'];
  dataSource: Coordinate[] = [];

  @ViewChild(MatTable) table: MatTable<Coordinate> | undefined;
  @ViewChild("inputFile") inputFile: ElementRef | undefined;
  @ViewChild("uploadImage") uploadImage: ElementRef | undefined;
  @ViewChild("robotArea") robotArea: ElementRef | undefined;
  @ViewChild("exportExcel") exportExcel: ElementRef | undefined;

  constructor(private httpService: HttpService,
              private sanitizer: DomSanitizer,
              private dialog: MatDialog,
              private storage: StorageService) { }

  ngOnInit(): void {
    this.getImage();

    this.storage.getCoordinateDelete().subscribe(id => {
      console.log('data = ' , id);
      this.dataSource.splice(id, 1);
      if (this.table) {
        this.table.renderRows();
      }
    });

    this.exportCoordinateUrl = this.httpService.getUrlExport();
  }

  ngAfterViewInit() {
    setTimeout(() => {
      this.getImageWidth();
    }, 500);
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
        this.image = this.sanitizer.bypassSecurityTrustResourceUrl(`data:${data.image.contentType};base64,${data.image.imageByte}`);
      } else if (data.status === 'ERROR') {
        this.message = data.message;
      }
    });
  }

  removeImage() {
    this.httpService.removeImage().subscribe((data: any) => {
      console.log(data);
      if (data.status === 'OK') {
        this.image = null;
        this.fileUpload = false;
        this.editingAllowed = true;
      }
      this.message = data.message;
      this.dataSource = [];
      this.dragImagePosition = {x: 0, y: 0};
    });
  }

  getImageWidth(): void {
      if (this.uploadImage) {
        this.imageWidth = this.uploadImage.nativeElement.width;
      }
  }

  renderImage(width: string) {
    if (width.length > this.maxWidthLen) {
      width = width.slice(0, this.maxWidthLen);
    }

    this.imageWidth = Number(width);
  }


  getDragImagePosition($event: CdkDragEnd) {
    const position = $event.source.getFreeDragPosition();
    this.dragImagePosition.x = position.x;
    this.dragImagePosition.y = position.y;
  }

  setEditingCompleted() {
    this.editingAllowed = false;
  }

  setEditAllowed() {
    this.editingAllowed = true;
  }

  saveCoordinate($event: MouseEvent) {
    const image = this.uploadImage?.nativeElement;
    let xPosition = 0;
    let yPosition = 0;

    let xScrollPos = image.scrollLeft || document.documentElement.scrollLeft;
    let yScrollPos = image.scrollTop || document.documentElement.scrollTop;

    xPosition += Math.round($event.x - (image.offsetLeft - xScrollPos + image.clientLeft));
    yPosition += Math.round($event.y - (image.offsetTop - yScrollPos + image.clientTop));

    this.dataSource.push({name: `${this.pointNameDefault}-${this.dataSource.length+1}`, x: xPosition, y: yPosition, z: 0});

    if (this.table) {
      this.table.renderRows();
    }
  }

  removeCoordinate(id: number) {
    this.dialog.open(OpenDialogComponent, {
      data: { id: id }
    });
  }

  changeCoordinateRow(value: any, type: string, id: number) {
    switch (type) {
      case 'x':
        this.dataSource[id].x = value;
        break;
      case 'y':
        this.dataSource[id].y = value;
        break;
      case 'z':
        this.dataSource[id].z = value;
        break;
      case 'name':
        this.dataSource[id].name = value;
        break;
    }

    if (this.table) {
      this.table.renderRows();
    }
  }

  exportFile() {
    this.exportExcel?.nativeElement.click();
  }

  coordinateSaveServer() {
    this.httpService.saveCoordinateToFile(this.dataSource).pipe().subscribe(() => {
      this.coordinateSaved = true;
    });
  }
}
