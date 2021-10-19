import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import {DownloadFileComponent} from "./download-file.component";
import {RouterModule} from "@angular/router";

@NgModule({
  declarations: [
    DownloadFileComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: DownloadFileComponent
    }])
  ],
  providers: [],
  bootstrap: [DownloadFileComponent]
})
export class DownloadFileModule { }
