import {Component, ElementRef, OnInit, Output, ViewChild} from '@angular/core';
import {FileHandle} from "./import-image/dragDrop.directive";
import {Coordinate, WorkOption} from "../../../model/models";
import {Subject} from "rxjs";
import {HttpService} from "../../../serviсes/http.service";
import {DomSanitizer} from "@angular/platform-browser";
import {MatDialog} from "@angular/material/dialog";
import {StorageService} from "../../../serviсes/storage.service";
import {Config} from "../../../config/config";
import {SizeService} from "../../../serviсes/size.service";
import {SessionService} from "../../../serviсes/session.service";
import {MessageService} from "../../../serviсes/message.service";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {Content} from "../../../content/content";

@UntilDestroy()
@Component({
  selector: 'app-command-lib',
  templateUrl: './command-lib.component.html',
  styleUrls: ['./command-lib.component.less'],
})
export class CommandLibComponent implements OnInit {
  title = 'docs';
  files: FileHandle[] = [];
  fileUpload: boolean = false;

  message: string | undefined;
  messageIsError: boolean = true;
  clickCoordinate: Coordinate | undefined;
  coordinateMessage: string = '';
  coordinateMessageError: boolean = false;

  currentStep: number = 1;

  sessionExists: boolean = false;

  imageUploaded: boolean = false;
  imageUploadRequired: boolean = true;

  editingAllowed: boolean = true;

  aboutImportOpen: boolean = false;

  workOptions: WorkOption[] = this.config.workOptions
  @Output() workOptionChecked: string = this.workOptions[0].key;

  messageImport: string = '';
  messageImportErrors: string[] = [];

  jsonInfo: string = '';
  choiceFileCommand: boolean = true; // Выбран файл с командами (иначе - с сеансом)

  @ViewChild("inputFilePoints") inputFilePoints: ElementRef | undefined;
  @ViewChild("inputFileSession") inputFileSession: ElementRef | undefined;

  public messages: Subject<any> | undefined;

  constructor(private httpService: HttpService,
              private sanitizer: DomSanitizer,
              private dialog: MatDialog,
              private config: Config,
              private storageService: StorageService,
              private sizeService: SizeService,
              private sessionService: SessionService,
              private messageService: MessageService,
              private content: Content) {
  }

  ngOnInit(): void {

    this.jsonInfo = this.content.jsonInfo;

    this.storageService.getCurrentStep().pipe(untilDestroyed(this)).subscribe(data => {
      this.currentStep = data;

      // текущие шаги запоминаем, только если есть сессия и картинка
      this.imageUploaded = data > 1;
    });

    this.sessionService.getSessionExists().pipe(untilDestroyed(this)).subscribe(data => this.sessionExists = data);

    this.sessionService.getImageEditAllowed().pipe(untilDestroyed(this)).subscribe(data => {
      this.editingAllowed = data;
    });

    this.sessionService.getWorkOptionKey().pipe(untilDestroyed(this)).subscribe(data => this.workOptionChecked = data);
    this.messageService.getMessageImport().pipe(untilDestroyed(this)).subscribe(data => this.messageImport = data);
    this.messageService.getMessageImportErrors().pipe(untilDestroyed(this)).subscribe(data => this.messageImportErrors = data);
  }

  setEditAllowed() {
    this.sessionService.setImageEditAllowed(true);
    this.storageService.setCurrentStep(2);
  }

  openDialogPointsFile() {
    if (this.inputFilePoints) {
      this.inputFilePoints.nativeElement.click();
    }
  }

  changeWorkOption(optionKey: string) {
    this.sessionService.setWorkOptionKey(optionKey);
  }

  changeFilePoints($event: Event) {
    const element = $event.currentTarget as HTMLInputElement;

    if (!element.files) {
      return;
    }

    this.choiceFileCommand = true;

    this.httpService.importCoordinates(element.files[0]).pipe(untilDestroyed(this)).subscribe(res => {
      this.messageService.setMessageImport(res.message);
      if (res.status === 'SUCCESS' || !res.details?.errorDetails) {
        this.messageService.setMessageImportErrors([]);
      } else {
        this.messageService.setMessageImportErrors(res.details.errorDetails);
      }

      if (res.details?.savedCoordinates) {
        this.sessionExists = true;
        this.sessionService.setSession(true);

        for (let item of res.details?.savedCoordinates) {
          this.sessionService.addCoordinateInList(item);
        }
      }
    })
  }

  openDialogSessionFile() {
    this.inputFileSession?.nativeElement?.click();
  }

  changeFileSession($event: Event) {
    const element = $event.currentTarget as HTMLInputElement;

    if (!element.files) {
      return;
    }

    this.choiceFileCommand = false;

    this.sessionService.importSession(element.files[0]);
  }
}
