import {Component, ElementRef, Input, OnInit, ViewChild} from '@angular/core';
import {StorageService} from "../../../../serviсes/storage.service";
import {Coordinate} from "../../../../model/models";
import {HttpService} from "../../../../serviсes/http.service";
import {MatTable} from "@angular/material/table";
import {WebsocketService} from "../../../../serviсes/websocket.service";
import {Config} from "../../../../config/config";
import {OpenDialogComponent} from "../../open-dialog/open-dialog.component";
import {MatDialog} from "@angular/material/dialog";
import {MessageService} from "../../../../serviсes/message.service";

@Component({
  selector: 'app-command-table',
  templateUrl: './command-table.component.html',
  styleUrls: ['./command-table.component.less']
})
export class CommandTableComponent implements OnInit {
  constructor(private storageService: StorageService,
              private httpService: HttpService,
              private wsService: WebsocketService,
              private config: Config,
              private dialog: MatDialog,
              private messageService: MessageService) {}

  coordinateList: Coordinate[] = [];

  validateError: boolean = true;
  fieldIdError: number | undefined; // поле с ошибкой
  coordValidateMessage: string = '';
  messageImport: string = '';
  messageImportErrors: string[] = [];

  clickCoordinate: Coordinate | undefined;
  selectedPointIndex: number | undefined;

  displayedColumns: string[] = ['name', 'x', 'y', 'z', 'action'];

  exportCoordinateUrl: string = '';
  exportTxtCoordinateUrl: string = '';

  @ViewChild(MatTable) table: MatTable<Coordinate> | undefined;
  @ViewChild("exportExcel") exportExcel: ElementRef | undefined;
  @ViewChild("exportTxt") exportTxt: ElementRef | undefined;

  ngOnInit(): void {

    this.storageService.getCoordinateList().subscribe(data => {

      if (data.length === 0) {
        this.clickCoordinate = undefined;
        this.selectedPointIndex = undefined;
      }

      this.coordinateList = data;
      this.renderTable();
    });

    this.storageService.getCoordinateDelete().subscribe(id => {

      const index = this.coordinateList.findIndex(c => c.id == id);
      this.coordinateList.splice(index, 1);
      this.renderTable();
      this.hideMessage();
    });

    this.wsService.connect(this.config.webSocketUrl).pipe().subscribe(res => {
      const response = JSON.parse(res.data);
      if (response.details) {
        const coordinate: Coordinate = response.details;
        this.coordinateList.push(coordinate);
        this.renderTable();

        this.validateError = response.status === 'ERROR';
        this.coordValidateMessage = response.message ? response.message : '';
        this.hideMessage();
      }
    });

    this.storageService.getClickCoordinate().subscribe(data => {
      if (data.id !== -1) {
        this.clickCoordinate = data;
      }
    });

    this.storageService.getCoordinateDeleteError().subscribe(err => this.validateError = err);
    this.storageService.getCoordinateDeleteMessage().subscribe(mess => this.coordValidateMessage = mess);
    this.exportCoordinateUrl = this.httpService.getUrlExport();
    this.exportTxtCoordinateUrl = this.httpService.getUrlExportTxt();

    this.messageService.getMessageImport().subscribe(data => {
      this.messageImport = data;
      this.hideMessage();
    });
    this.messageService.getMessageImportErrors().subscribe(data => { this.messageImportErrors = data });
  }

  changeCoordinateRow(value: any, type: string, id: number) {

    const coordinate = this.coordinateList.filter(c => c.id === id).shift();

    if (!coordinate) {
      return;
    }

    const coordinateNamePrev = coordinate.name;

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

      const index = this.coordinateList.findIndex(c => c.id === coordinate.id);
      if (!this.validateError) {
        this.coordinateList[index] = coordinate;
        this.fieldIdError = undefined;

      } else {
        this.fieldIdError = coordinate.id;
        this.coordinateList[index].name = coordinateNamePrev;
      }

      this.hideMessage();
      this.renderTable();
    });
  }

  removeCoordinate(id: number) {
    this.dialog.open(OpenDialogComponent, {
      data: { id: id, title: 'Удалить строку?', text: 'Действие отменить нельзя.', type: 'delete_coordinate' }
    });
  }

  showClickPoint(point: Coordinate, index: number) {
    this.storageService.setClickCoordinate(point);
    this.selectedPointIndex = index;
  }

  renderTable(): void {
    this.clearImportMessage();
    if (this.table) {
      this.table.renderRows();
    }
  }

  removeAllPoint() {
    this.clearImportMessage();
    this.dialog.open(OpenDialogComponent, {
      data: {title: 'Удалить все координаты?',
        text: 'Действие отменить нельзя.',
        type: 'remove_all_coordinates'}
    });
  }

  exportFile() {
    this.clearImportMessage();
    this.exportExcel?.nativeElement.click();
  }

  exportFileTxt() {
    this.clearImportMessage();
    this.exportTxt?.nativeElement.click();
  }

  hideMessage() {
    if (!this.validateError) {
      setTimeout(() => {
        this.coordValidateMessage = '';
      }, 3000);
    } else if (this.messageImportErrors.length === 0) {
      setTimeout(() => {
        this.messageImport = '';
      }, 3000);
    }
  }

  clearImportMessage() {
    this.messageService.setMessageImport('');
    this.messageService.setMessageImportErrors([]);
  }
}
