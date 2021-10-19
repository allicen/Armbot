import { NgModule } from '@angular/core';

import {RosComponent} from "./ros.component";
import {RouterModule} from "@angular/router";
import {ViewService} from "../../../serviсes/view.service";

@NgModule({
  declarations: [
    RosComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: RosComponent
    }])
  ],
  exports: [
    RosComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [RosComponent]
})
export class RosModule { }
