import { NgModule } from '@angular/core';

import {GeneralComponent} from "./general.component";
import {RouterModule} from "@angular/router";
import {ViewService} from "../../serviсes/view.service";

@NgModule({
  declarations: [
    GeneralComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: GeneralComponent
    }])
  ],
  exports: [
    GeneralComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [GeneralComponent]
})
export class GeneralModule { }
