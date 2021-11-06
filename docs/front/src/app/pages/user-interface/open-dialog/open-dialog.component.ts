import {Component, Inject, OnInit} from '@angular/core';
import {MAT_DIALOG_DATA, MatDialogRef} from "@angular/material/dialog";
import {IdData} from "../../../model/models";
import {StorageService} from "../../../serviсes/storage.service";
import {HttpService} from "../../../serviсes/http.service";
import {MessageService} from "../../../serviсes/message.service";

@Component({
  selector: 'app-open-dialog',
  templateUrl: './open-dialog.component.html',
  styleUrls: ['./open-dialog.component.less']
})
export class OpenDialogComponent implements OnInit {
  constructor(public dialogRef: MatDialogRef<OpenDialogComponent>,
              @Inject(MAT_DIALOG_DATA) public data: IdData,
              private storageService: StorageService,
              private httpService: HttpService,
              private messageService: MessageService) {}

  title: string = '';
  text: string = '';
  type: string = '';

  ngOnInit(): void {
    this.title = this.data.title;
    this.text = this.data.text;
    this.type = this.data.type;
  }

  deleteRow() {
    this.storageService.setCoordinateDelete(this.data.id);
    this.dialogRef.close();
  }

  removeSession() {
    this.storageService.setRemoveSession(true);
    this.dialogRef.close();
  }

  removeAllRows() {
    this.httpService.removeAllCoordinates().pipe().subscribe((res) => {

      if (res.status === 'SUCCESS') {
        this.storageService.setCoordinateList([]);
      }

      this.messageService.setCoordinateMessage(res.message);
      this.messageService.setCoordinateMessageIsError(res.message === 'ERROR');
    });
    this.dialogRef.close();
  }
}
