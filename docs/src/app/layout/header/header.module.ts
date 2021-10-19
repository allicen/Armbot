import { NgModule } from '@angular/core';

import {HeaderComponent} from "./header.component";
import {RouterModule} from "@angular/router";
import {MatSlideToggleModule} from "@angular/material/slide-toggle";
import {FormsModule} from "@angular/forms";

@NgModule({
  declarations: [
    HeaderComponent
  ],
    imports: [
        RouterModule,
        MatSlideToggleModule,
        FormsModule
    ],
  exports: [
    HeaderComponent
  ],
  bootstrap: [HeaderComponent]
})
export class HeaderModule { }
