import {Component, Inject, OnInit} from '@angular/core';
import {MAT_DIALOG_DATA, MatDialogRef} from "@angular/material/dialog";
import {IdData} from "../../../model/models";
import {StorageService} from "../../../serviсes/storage.service";
import {HttpService} from "../../../serviсes/http.service";
import {MessageService} from "../../../serviсes/message.service";
import {SessionService} from "../../../serviсes/session.service";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";

@UntilDestroy()
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
              private messageService: MessageService,
              private imageService: SessionService,
              private sessionService: SessionService) {}

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
    this.sessionService.removeSession();
    this.dialogRef.close();
  }

  removeAllRows() {
    this.httpService.removeAllCoordinates().pipe(untilDestroyed(this)).subscribe((res) => {

      if (res.status === 'SUCCESS') {
        this.sessionService.setCoordinateList([]);
        this.sessionService.setLaunchFileRow([]);
      }

      this.messageService.setCoordinateMessage(res.message);
      this.messageService.setCoordinateMessageIsError(res.message === 'ERROR');
    });
    this.dialogRef.close();
  }
}
