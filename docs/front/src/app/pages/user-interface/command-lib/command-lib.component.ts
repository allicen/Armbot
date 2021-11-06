import {Component, ElementRef, OnInit, ViewChild, ViewEncapsulation} from '@angular/core';
import {FileHandle} from "./import-image/dragDrop.directive";
import {Coordinate, Position} from "../../../model/models";
import {MatTable} from "@angular/material/table";
import {Subject} from "rxjs";
import {HttpService} from "../../../serviсes/http.service";
import {DomSanitizer} from "@angular/platform-browser";
import {MatDialog} from "@angular/material/dialog";
import {StorageService} from "../../../serviсes/storage.service";
import {WebsocketService} from "../../../serviсes/websocket.service";
import {Config} from "../../../config/config";
import {SizeService} from "../../../serviсes/size.service";
import {OpenDialogComponent} from "../open-dialog/open-dialog.component";
import {ImageService} from "../../../serviсes/image.service";
import {delay} from "rxjs/operators";
import {MessageService} from "../../../serviсes/message.service";

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
  clickCoordinate: Coordinate | undefined;

  currentStep: number = 1;

  image: any = null;
  dragImagePosition: Position = {x: 0, y: 0};
  imageUploadRequired: boolean = true;

  editingAllowed: boolean = true;

  aboutImportOpen: boolean = false;

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

    this.imageService.getImagePosition().subscribe(data => {
      this.dragImagePosition = data;
    });

    this.imageService.getImageEditAllowed().subscribe(data => {
      this.editingAllowed = data;
    });

    this.imageService.getImage().subscribe(data => {
      this.image = data;
      this.storageService.setCurrentStep(2);
    });

    this.imageService.getImageUploadRequired().subscribe(data => {
      this.imageUploadRequired = data;
    });

    this.storageService.getCurrentStep().subscribe(data => {
      this.currentStep = data;
    });
  }

  getSession() {
    // return this.httpService.getSession().subscribe((data: any) => {
    //   if (data.status === 'SUCCESS' && data.details) {
    //     const details = data.details.sessionState;
    //     this.dragImagePosition = {x: details.imagePositionX, y: details.imagePositionY};
    //
    //     if (data.details.coordinateList) {
    //       this.dataSource = data.details.coordinateList;
    //     }
    //   }
    // });
  }

  removeImage() {
    this.httpService.removeImage().subscribe((data: any) => {
      if (data.status === 'OK') {
        this.fileUpload = false;
      }
      this.message = data.message;
      //this.dataSource = [];

      this.imageService.setImagePosition(0, 0);

    });

    this.files = [];
  }

  setEditAllowed() {
    this.imageService.setImageEditAllowed(true);
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
    const element = $event.currentTarget as HTMLInputElement;

    if (!element.files) {
      return;
    }

    this.httpService.importCoordinates(element.files[0]).subscribe(res => {
      this.messageService.setMessageImport(res.message);
      if (res.status === 'SUCCESS' || !res.details?.errorDetails) {
        this.messageService.setMessageImportErrors([]);
      } else {
        this.messageService.setMessageImportErrors(res.details.errorDetails);
      }

      if (res.details?.savedCoordinates) {
        for (let item of res.details?.savedCoordinates) {
          this.dataSource.push(item);
          this.storageService.setCoordinateList(this.dataSource);
        }
      }
    })
  }


}
