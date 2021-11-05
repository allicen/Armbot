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
  dragImagePosition: Position = {x: 0, y: 0};
  clickCoordinate: Coordinate | undefined;
  selectedPointIndex: number | undefined;
  exportCoordinateUrl: string = '';
  exportTxtCoordinateUrl: string = '';

  currentStep: number = 1;

  image: any = null;

  editingAllowed: boolean = true;

  displayedColumns: string[] = ['name', 'x', 'y', 'z', 'action'];
  dataSource: Coordinate[] = [];

  coordValidateMessage: string = '';
  importMessage: string = '';
  importErrors: [] | undefined;
  validateError: boolean = true;
  fieldIdError: number | undefined; // поле с ошибкой

  @ViewChild(MatTable) table: MatTable<Coordinate> | undefined;
  @ViewChild("inputFilePoints") inputFilePoints: ElementRef | undefined;
  @ViewChild("exportExcel") exportExcel: ElementRef | undefined;
  @ViewChild("exportTxt") exportTxt: ElementRef | undefined;

  public messages: Subject<any> | undefined;

  constructor(private httpService: HttpService,
              private sanitizer: DomSanitizer,
              private dialog: MatDialog,
              private storage: StorageService,
              private wsService: WebsocketService,
              private config: Config,
              private sizeService: SizeService,
              private imageService: ImageService) {

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

    this.imageService.getImagePosition().subscribe(data => {
      this.dragImagePosition = data;
    });

    this.storage.getClickCoordinate().subscribe(data => {
      if (data.id !== -1) {
        this.clickCoordinate = data;
      }
    });

    this.imageService.getImageEditAllowed().subscribe(data => {
      this.editingAllowed = data;
    });

    this.imageService.getImage().subscribe(data => {
      this.image = data;
    });
  }

  getSession() {
    return this.httpService.getSession().subscribe((data: any) => {
      if (data.status === 'SUCCESS' && data.details) {
        const details = data.details.sessionState;
        this.dragImagePosition = {x: details.imagePositionX, y: details.imagePositionY};

        if (data.details.coordinateList) {
          this.dataSource = data.details.coordinateList;
        }
      }
    });
  }

  removeImage() {
    this.httpService.removeImage().subscribe((data: any) => {
      if (data.status === 'OK') {
        this.fileUpload = false;
      }
      this.message = data.message;
      this.dataSource = [];

      this.imageService.setImagePosition(0, 0);

      this.importMessage = '';
      this.importErrors= [];
    });

    this.files = [];
  }

  setEditAllowed() {
    this.imageService.setImageEditAllowed(true);
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

      const index = this.dataSource.findIndex(c => c.id === coordinate.id);
      if (!this.validateError) {
        this.dataSource[index] = coordinate;
        this.fieldIdError = undefined;

      } else {
        this.fieldIdError = coordinate.id;
      }

      this.hideMessage();
      this.importMessage = '';
      this.importErrors= [];

      if (this.table) {
        this.table.renderRows();
      }
    });
  }

  exportFile() {
    this.exportExcel?.nativeElement.click();
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
    this.storage.setClickCoordinate(point);
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


  openDialogPointsFile($event: MouseEvent) {
    if (this.inputFilePoints) {
      this.inputFilePoints.nativeElement.click();
    }
  }

  changeFilePoints($event: Event) {
    const element = $event.currentTarget as HTMLInputElement;

    if (!element.files) {
      return;
    }

    this.httpService.importCoordinates(element.files[0]).subscribe(res => {
      if (res.status === 'SUCCESS') {
        this.importMessage = res.message;
        this.validateError = false;
      } else {
        this.validateError = true;
        this.importMessage = res.message;
        if (res.details?.errorDetails) {
          this.importErrors = res.details.errorDetails
        }
      }

      if (res.details?.savedCoordinates) {

        for (let item of res.details?.savedCoordinates) {
          this.dataSource.push(item);
        }
      }

      if (this.table) {
        this.table.renderRows();
      }
    })
  }
}
