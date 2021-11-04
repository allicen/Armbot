import { NgModule } from '@angular/core';

import {CommandLibComponent} from "./command-lib.component";
import {RouterModule} from "@angular/router";
import {DragDirective} from "./dragDrop.directive";
import {OpenDialogComponent} from "../open-dialog/open-dialog.component";
import {CommonModule} from "@angular/common";
import {MatButtonModule} from "@angular/material/button";
import {MatInputModule} from "@angular/material/input";
import {DragDropModule} from "@angular/cdk/drag-drop";
import {MatTableModule} from "@angular/material/table";
import {MatIconModule} from "@angular/material/icon";
import {MatDialog, MatDialogModule} from "@angular/material/dialog";
import {MatCheckboxModule} from "@angular/material/checkbox";
import {MatRadioModule} from "@angular/material/radio";
import {FormsModule, ReactiveFormsModule} from "@angular/forms";
import {HttpClient} from "@angular/common/http";
import {MatSnackBar} from "@angular/material/snack-bar";
import {WebsocketService} from "../../../serviсes/websocket.service";

@NgModule({
  declarations: [
    CommandLibComponent,
    DragDirective,
    OpenDialogComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: CommandLibComponent
    }]),
    CommonModule,
    MatButtonModule,
    MatInputModule,
    DragDropModule,
    MatTableModule,
    MatIconModule,
    MatDialogModule,
    MatCheckboxModule,
    MatRadioModule,
    FormsModule,
    ReactiveFormsModule
  ],
  providers: [HttpClient, MatDialog, MatSnackBar, WebsocketService],
  bootstrap: [CommandLibComponent]
})
export class CommandLibModule { }
