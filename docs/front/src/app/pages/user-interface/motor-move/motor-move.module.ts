import { NgModule } from '@angular/core';
import {MotorMoveComponent} from "./motor-move.component";
import {RouterModule} from "@angular/router";
import {MatListModule} from "@angular/material/list";
import {CommonModule} from "@angular/common";
import {MatFormFieldModule} from "@angular/material/form-field";
import {MatChipsModule} from "@angular/material/chips";
import {MatAutocompleteModule} from "@angular/material/autocomplete";
import {MatOptionModule} from "@angular/material/core";
import {MatIconModule} from "@angular/material/icon";
import {FormsModule, ReactiveFormsModule} from "@angular/forms";
import {DragDropModule} from "@angular/cdk/drag-drop";
import {MatButtonModule} from "@angular/material/button";
import {MatInputModule} from "@angular/material/input";
import {MatRadioModule} from "@angular/material/radio";

@NgModule({
  declarations: [
    MotorMoveComponent,
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: MotorMoveComponent
    }]),
    MatListModule,
    CommonModule,
    MatFormFieldModule,
    MatChipsModule,
    MatAutocompleteModule,
    MatOptionModule,
    MatIconModule,
    ReactiveFormsModule,
    DragDropModule,
    MatButtonModule,
    MatInputModule,
    MatRadioModule,
    FormsModule,
  ],
  exports: [],
  providers: [],
  bootstrap: []
})
export class MotorMoveModule { }
