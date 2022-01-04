import { NgModule } from '@angular/core';

import {HeaderComponent} from "./header.component";
import {RouterModule} from "@angular/router";
import {MatSlideToggleModule} from "@angular/material/slide-toggle";
import {FormsModule} from "@angular/forms";
import {MatButtonModule} from "@angular/material/button";
import {CommonModule} from "@angular/common";
import {MatIconModule} from "@angular/material/icon";

@NgModule({
  declarations: [
    HeaderComponent
  ],
  imports: [
    RouterModule,
    MatSlideToggleModule,
    FormsModule,
    MatButtonModule,
    CommonModule,
    MatIconModule
  ],
  exports: [
    HeaderComponent
  ],
  bootstrap: [HeaderComponent]
})
export class HeaderModule { }
