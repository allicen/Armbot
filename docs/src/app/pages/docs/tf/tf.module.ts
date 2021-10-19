import { NgModule } from '@angular/core';

import {TfComponent} from "./tf.component";
import {RouterModule} from "@angular/router";
import {ViewService} from "../../../serviсes/view.service";

@NgModule({
  declarations: [
    TfComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: TfComponent
    }])
  ],
  exports: [
    TfComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [TfComponent]
})
export class TfModule { }
