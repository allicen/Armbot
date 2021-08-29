import { NgModule } from '@angular/core';

import {FooterComponent} from "./footer.component";
import {RouterModule} from "@angular/router";

@NgModule({
  declarations: [
    FooterComponent
  ],
  imports: [
    RouterModule
  ],
  exports: [
    FooterComponent
  ],
  bootstrap: [FooterComponent]
})
export class FooterModule { }
