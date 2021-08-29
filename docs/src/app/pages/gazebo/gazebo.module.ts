import { NgModule } from '@angular/core';

import {GazeboComponent} from "./gazebo.component";
import {RouterModule} from "@angular/router";
import {ViewService} from "../../serviсes/view.service";

@NgModule({
  declarations: [
    GazeboComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: GazeboComponent
    }])
  ],
  exports: [
    GazeboComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [GazeboComponent]
})
export class GazeboModule { }
