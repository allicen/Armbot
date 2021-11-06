import {Component, ElementRef, OnInit, ViewChild} from '@angular/core';
import {FileHandle} from "./import-image/dragDrop.directive";
import {Coordinate} from "../../../model/models";
import {Subject} from "rxjs";
import {HttpService} from "../../../serviсes/http.service";
import {DomSanitizer} from "@angular/platform-browser";
import {MatDialog} from "@angular/material/dialog";
import {StorageService} from "../../../serviсes/storage.service";
import {Config} from "../../../config/config";
import {SizeService} from "../../../serviсes/size.service";
import {OpenDialogComponent} from "../open-dialog/open-dialog.component";
import {ImageService} from "../../../serviсes/image.service";
import {MessageService} from "../../../serviсes/message.service";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";

@UntilDestroy()
@Component({
  selector: 'app-command-lib',
  templateUrl: './command-lib.component.html',
  styleUrls: ['./command-lib.component.less'],
})
export class CommandLibComponent implements OnInit {
  title = 'docs';
  files: FileHandle[] = [];
  dataSource: Coordinate[] = [];
  fileUpload: boolean = false;

  message: string | undefined;
  messageIsError: boolean = true;
  clickCoordinate: Coordinate | undefined;
  coordinateMessage: string = '';
  coordinateMessageError: boolean = false;

  currentStep: number = 1;

  image: any = null;
  imageUploadRequired: boolean = true;

  editingAllowed: boolean = true;

  aboutImportOpen: boolean = false;

  sessionStart: boolean = false;
  sessionNotFound: boolean = true;

  @ViewChild("inputFilePoints") inputFilePoints: ElementRef | undefined;

  public messages: Subject<any> | undefined;

  constructor(private httpService: HttpService,
              private sanitizer: DomSanitizer,
              private dialog: MatDialog,
              private config: Config,
              private storageService: StorageService,
              private sizeService: SizeService,
              private imageService: ImageService,
              private messageService: MessageService) {
  }

  ngOnInit(): void {
    this.getSession();

    this.imageService.getImageEditAllowed().subscribe(data => {
      this.editingAllowed = data;
    });

    this.imageService.getImage().pipe(untilDestroyed(this)).subscribe(data => {
      this.image = data;
      if (this.image && this.editingAllowed) {
        this.storageService.setCurrentStep(2);
        this.storageService.setSessionStart(true);
        this.sessionNotFound = false;
      }
    });

    this.imageService.getImageUploadRequired().pipe(untilDestroyed(this)).subscribe(data => {
      this.imageUploadRequired = data;
    });

    this.storageService.getCurrentStep().pipe(untilDestroyed(this)).subscribe(data => {
      this.currentStep = data;
    });

    this.storageService.getSessionStart().pipe(untilDestroyed(this)).subscribe(start => {
      this.sessionStart = start;
    });

    this.storageService.getSessionRemove().pipe(untilDestroyed(this)).subscribe(remove => {
      if (remove) {
        this.removeSessionAccept();
      }
    });

    this.messageService.getCoordinateMessage().pipe(untilDestroyed(this)).subscribe(data => this.coordinateMessage = data);
    this.messageService.getCoordinateMessageIsError().pipe(untilDestroyed(this)).subscribe(data => {
      this.coordinateMessageError = data;
      if (!data) {
        setTimeout(() => {
          this.coordinateMessage = '';
        }, 3000);
      }
    });
  }

  getSession() {
    return this.httpService.getSession().pipe(untilDestroyed(this)).subscribe((data: any) => {

      if (data.status === 'NO_SESSION') {
        this.storageService.setSessionStart(false);
      }

      if (data.status === 'SUCCESS' && data.details) {
        const details = data.details.sessionState;
        this.imageService.setImagePosition(details.imagePositionX, details.imagePositionY);
        this.imageService.setImageWidth(details.imageSize);
        this.imageService.setImageEditAllowed(false);
        this.storageService.setCurrentStep(3);
        this.sessionStart = true;

        if (data.details.coordinateList) {
          this.storageService.setCoordinateList(data.details.coordinateList);
        }

        this.imageService.setImageUploadRequired(data.details.sessionState.imageRequired);

        this.sessionNotFound = false;
      }
    });
  }

  removeSession() {
    this.clearImportMessage();
    this.dialog.open(OpenDialogComponent, {
      data: {title: 'Завершить сеанс?',
            text: 'Будут удалены все координаты и загруженное изображение. Действие отменить нельзя.',
            type: 'remove_session'}
    });
  }

  removeSessionAccept() {
    this.httpService.removeSession().pipe(untilDestroyed(this)).subscribe((data: any) => {

      this.message = data.message;

      if (data.status === 'SUCCESS') {
        this.fileUpload = false;
        this.messageIsError = false;

        setTimeout(() => {
          this.message = '';
        }, 3000);

        this.imageService.deleteImage();
        this.storageService.setCurrentStep(1);
        this.imageService.setImageEditAllowed(true);
        this.imageService.setImageWidth(0);
        this.imageService.setImagePosition(0, 0);
        this.storageService.setCoordinateList([]);

      } else {
        this.messageIsError = true;
      }
    });

    this.files = [];
  }

  setEditAllowed() {
    this.clearImportMessage();
    this.imageService.setImageEditAllowed(true);
    this.storageService.setCurrentStep(2);
  }

  openDialogPointsFile($event: MouseEvent) {
    if (this.inputFilePoints) {
      this.inputFilePoints.nativeElement.click();
    }
  }

  changeWorkOption() {
    this.imageService.setImageUploadRequired(!this.imageUploadRequired);
  }

  changeFilePoints($event: Event) {
    this.clearImportMessage();
    const element = $event.currentTarget as HTMLInputElement;

    if (!element.files) {
      return;
    }

    this.httpService.importCoordinates(element.files[0]).pipe(untilDestroyed(this)).subscribe(res => {
      this.messageService.setMessageImport(res.message);
      if (res.status === 'SUCCESS' || !res.details?.errorDetails) {
        this.messageService.setMessageImportErrors([]);
      } else {
        this.messageService.setMessageImportErrors(res.details.errorDetails);
      }

      if (res.details?.savedCoordinates) {
        for (let item of res.details?.savedCoordinates) {
          this.storageService.addCoordinateInList(item);
        }

        this.storageService.setSessionStart(true);

        if (!this.imageUploadRequired) {
          this.httpService.saveSessionState(-1, {x: 0, y: 0}, this.imageUploadRequired)
            .pipe(untilDestroyed(this))
            .subscribe(res => {

          });
        }
      }
    })
  }

  clearImportMessage() {
    this.messageService.setMessageImport('');
    this.messageService.setMessageImportErrors([]);
    this.messageService.setCoordinateMessage('');
    this.messageService.setCoordinateMessageIsError(false);
  }


}
