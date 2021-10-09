import { NgModule } from '@angular/core';

import {ErrorsComponent} from "./errors.component";
import {RouterModule} from "@angular/router";
import {ViewService} from "../../serviсes/view.service";

@NgModule({
  declarations: [
    ErrorsComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: ErrorsComponent
    }])
  ],
  exports: [
    ErrorsComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [ErrorsComponent]
})
export class ErrorsModule { }
