import { NgModule } from '@angular/core';

import {LinuxComponent} from "./linux.component";
import {RouterModule} from "@angular/router";
import {ViewService} from "../../serviсes/view.service";

@NgModule({
  declarations: [
    LinuxComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: LinuxComponent
    }])
  ],
  exports: [
    LinuxComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [LinuxComponent]
})
export class LinuxModule { }
