import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import {UploadFileComponent} from "./upload-file.component";
import {RouterModule} from "@angular/router";

@NgModule({
  declarations: [
    UploadFileComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: UploadFileComponent
    }])
  ],
  providers: [],
  bootstrap: [UploadFileComponent]
})
export class UploadFileModule { }
