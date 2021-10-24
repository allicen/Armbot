import {Component, Inject, OnInit, ViewEncapsulation} from '@angular/core';
import {MAT_DIALOG_DATA, MatDialogRef} from "@angular/material/dialog";
import {IdData} from "../../../model/models";
import {StorageService} from "../../../serviсes/storage.service";

@Component({
  selector: 'app-open-dialog',
  templateUrl: './open-dialog.component.html',
  styleUrls: ['./open-dialog.component.less']
})
export class OpenDialogComponent implements OnInit {
  constructor(public dialogRef: MatDialogRef<OpenDialogComponent>,
              @Inject(MAT_DIALOG_DATA) public data: IdData, private storageService: StorageService) {}

  ngOnInit(): void {
    console.log(this.data);
  }

  deleteRow() {
    this.storageService.setCoordinateDelete(this.data.id);
    this.dialogRef.close();
  }
}
