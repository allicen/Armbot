import { NgModule } from '@angular/core';

import {LaunchComponent} from "./launch.component";
import {RouterModule} from "@angular/router";
import {ViewService} from "../../serviсes/view.service";

@NgModule({
  declarations: [
    LaunchComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: LaunchComponent
    }])
  ],
  exports: [
    LaunchComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [LaunchComponent]
})
export class LaunchModule { }
