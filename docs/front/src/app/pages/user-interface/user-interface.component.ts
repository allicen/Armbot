import {Component, ElementRef, Injectable, OnInit, ViewChild} from '@angular/core';
import { FileHandle } from './dragDrop.directive';
import {DomSanitizer} from "@angular/platform-browser";
import {HttpService} from "../../serviсes/http.service";
import {HttpClient} from "@angular/common/http";
import { CdkDragEnd } from "@angular/cdk/drag-drop";

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


  @ViewChild("inputFile") inputFile: ElementRef | undefined;
  @ViewChild("uploadImage") uploadImage: ElementRef | undefined;

  constructor(private httpService: HttpService,
              private sanitizer: DomSanitizer) { }

  ngOnInit(): void {
    this.getImage();
  }

  ngAfterViewInit() {
    setTimeout(() => {
      this.getImageWidth();
    }, 100);
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

  saveCoordinate() {

  }
}
