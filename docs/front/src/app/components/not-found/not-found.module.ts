import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import {NotFoundComponent} from "./not-found.component";
import {RouterModule} from "@angular/router";

@NgModule({
  declarations: [
    NotFoundComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: NotFoundComponent
    }])
  ],
  providers: [],
  bootstrap: [NotFoundComponent]
})
export class NotFoundModule { }
