import {Component, ElementRef, ViewChild} from '@angular/core';
import { FileHandle } from './dragDrop.directive';
import {DomSanitizer} from "@angular/platform-browser";
import {HttpService} from "../../serviсes/http.service";
import {HttpClient} from "@angular/common/http";

@Component({
  selector: 'app-root',
  templateUrl: './user-interface.component.html',
  styleUrls: ['./user-interface.component.less']
})
export class UserInterfaceComponent {
  title = 'docs';
  files: FileHandle[] = [];
  fileUpload: boolean = false;

  @ViewChild("inputFile") inputFile: ElementRef | undefined;

  constructor(private sanitizer: DomSanitizer, private httpService: HttpService) { }

  filesDropped(files: FileHandle[]): void {
    if (files.length > 0) {
      // поддерживается загрузка несколльких файлов
      // загружаем только 1й файл!
      this.files = files.slice(0, 1);
      this.fileUpload = true;
    }
  }

  uploadFile(): void {
    // загрузка файла;

    if (this.files.length === 0) {
      return;
    }

    this.httpService.uploadImage(this.files[0]);
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
}
