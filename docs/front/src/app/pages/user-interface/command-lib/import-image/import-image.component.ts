import {Component, ElementRef, Input, OnInit, ViewChild} from '@angular/core';
import {FileHandle} from "./dragDrop.directive";
import {DomSanitizer} from "@angular/platform-browser";
import {HttpService} from "../../../../serviсes/http.service";
import {ImageService} from "../../../../serviсes/image.service";
import {Config} from "../../../../config/config";

@Component({
  selector: 'app-import-image',
  templateUrl: './import-image.component.html',
  styleUrls: ['./import-image.component.less']
})
export class ImportImageComponent implements OnInit {
  constructor(private sanitizer: DomSanitizer,
              private httpService: HttpService,
              private imageService: ImageService,
              private config: Config) {}

  message: string = '';
  messageIsError = false;
  types: string = '';

  files: FileHandle[] = [];
  fileUpload: boolean = false;
  @ViewChild("inputFile") inputFile: ElementRef | undefined;

  ngOnInit(): void {
    this.types = this.config.allowImageMimeTypes.join(', ');
  }

  filesDropped(files: FileHandle[]): void {
    if (files.length > 0) {
      // поддерживается загрузка несколльких файлов
      // загружаем только 1й файл!
      this.files = files.slice(0, 1);
      this.fileUpload = true;
    }
  }

  openDialogChangeFile($event: Event) {
    if (this.inputFile) {
      this.inputFile.nativeElement.click();
    }
  }

  changeFile($event: Event) {
    const element = $event.currentTarget as HTMLInputElement;
    let fileList: FileList | null = element.files;
    this.messageClear();

    if (fileList) {
      const file = fileList[0];

      console.log(file);

      if (this.config.allowImageMimeTypes.includes(file.type) && file.size < this.config.imageMaxSize) {
          const url = this.sanitizer.bypassSecurityTrustUrl(window.URL.createObjectURL(file));
          this.files.push({ file, url });
          this.message = 'Выбранное изображение можно загрузить';

      } else if (this.config.allowImageMimeTypes.includes(file.type)) {
        this.message = `Размер изображения (${file.size} байт) больше разрешенного: ${this.config.imageMaxSize} байт`;
        this.messageIsError = true;
      } else {
        this.message = `Тип файла '${file.type}' не поддерживается! Поддерживаемые типы: ${this.types}`;
        this.messageIsError = true;
      }
    }

    if (this.files.length > 0) {
      this.fileUpload = true;
    }
  }

  removeFile() {
    this.files = [];
    this.fileUpload = false;
    this.messageClear();
  }

  uploadFile(): any {
    if (this.files.length === 0) {
      return;
    }

    this.imageService.setImage(this.files[0]);
    this.messageClear();
  }

  messageClear() {
    this.message = '';
    this.messageIsError = false;
  }
}