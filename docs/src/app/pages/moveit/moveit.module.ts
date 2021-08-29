import { NgModule } from '@angular/core';

import {MoveitComponent} from "./moveit.component";
import {RouterModule} from "@angular/router";
import {ViewService} from "../../serviсes/view.service";

@NgModule({
  declarations: [
    MoveitComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: MoveitComponent
    }])
  ],
  exports: [
    MoveitComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [MoveitComponent]
})
export class MoveitModule { }
