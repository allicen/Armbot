import { NgModule } from '@angular/core';

import {MoreComponent} from "./more.component";
import {RouterModule} from "@angular/router";
import {ViewService} from "../../serviсes/view.service";

@NgModule({
  declarations: [
    MoreComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: MoreComponent
    }])
  ],
  exports: [
    MoreComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [MoreComponent]
})
export class MoreModule { }
