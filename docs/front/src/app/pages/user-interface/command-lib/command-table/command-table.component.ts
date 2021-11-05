import {Component, ElementRef, Input, OnInit, ViewChild} from '@angular/core';
import {StorageService} from "../../../../serviсes/storage.service";
import {Coordinate} from "../../../../model/models";
import {HttpService} from "../../../../serviсes/http.service";
import {MatTable} from "@angular/material/table";
import {WebsocketService} from "../../../../serviсes/websocket.service";
import {Config} from "../../../../config/config";
import {OpenDialogComponent} from "../../open-dialog/open-dialog.component";
import {MatDialog} from "@angular/material/dialog";

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
              private dialog: MatDialog,) {}

  coordinateList: Coordinate[] = [];

  validateError: boolean = true;
  fieldIdError: number | undefined; // поле с ошибкой
  coordValidateMessage: string = '';
  importMessage: string = '';
  importErrors: [] | undefined;
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

      console.log('data = ', data);

      this.coordinateList = data;
      this.renderTable();
    });

    this.storageService.getCoordinateDelete().subscribe(id => {

      const index = this.coordinateList.findIndex(c => c.id == id);
      this.coordinateList.splice(index, 1);
      this.renderTable();

      // this.hideMessage();
    });

    this.wsService.connect(this.config.webSocketUrl).pipe().subscribe(res => {
      const response = JSON.parse(res.data);
      if (response.details) {
        const coordinate: Coordinate = response.details;
        this.coordinateList.push(coordinate);
        this.renderTable();

        this.validateError = response.status === 'ERROR';
        this.coordValidateMessage = response.message ? response.message : '';
        // this.hideMessage();
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
  }

  changeCoordinateRow(value: any, type: string, id: number) {

    const coordinate = this.coordinateList.filter(c => c.id === id).shift();

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

      const index = this.coordinateList.findIndex(c => c.id === coordinate.id);
      if (!this.validateError) {
        this.coordinateList[index] = coordinate;
        this.fieldIdError = undefined;

      } else {
        this.fieldIdError = coordinate.id;
      }

      // this.hideMessage();
      this.importMessage = '';
      this.importErrors= [];

      this.renderTable();
    });
  }

  removeCoordinate(id: number) {
    this.dialog.open(OpenDialogComponent, {
      data: { id: id }
    });
  }

  showClickPoint(point: Coordinate, index: number) {
    this.storageService.setClickCoordinate(point);
    this.selectedPointIndex = index;
  }

  renderTable(): void {
    if (this.table) {
      this.table.renderRows();
    }
  }

  removeAllPoint() {
    this.httpService.removeAllCoordinates().pipe().subscribe((res) => {
      if (res.status === 'SUCCESS') {
        // this.coordinateList = [];
        this.clickCoordinate = undefined;
        // this.selectedPointIndex = undefined;
      }
    });
  }

  exportFile() {
    this.exportExcel?.nativeElement.click();
  }

  exportFileTxt() {
    this.exportTxt?.nativeElement.click();
  }
}
