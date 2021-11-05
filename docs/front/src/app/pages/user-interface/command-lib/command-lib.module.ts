import { NgModule } from '@angular/core';

import {CommandLibComponent} from "./command-lib.component";
import {RouterModule} from "@angular/router";
import {DragDirective} from "./import-image/dragDrop.directive";
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
import {ImportImageComponent} from "./import-image/import-image.component";
import {RobotAreaComponent} from "./robot-area/robot-area.component";
import {CommandTableComponent} from "./command-table/command-table.component";

@NgModule({
  declarations: [
    CommandLibComponent,
    DragDirective,
    ImportImageComponent,
    RobotAreaComponent,
    CommandTableComponent
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
  exports: [
    CommandLibComponent
  ],
  bootstrap: [CommandLibComponent]
})
export class CommandLibModule { }
