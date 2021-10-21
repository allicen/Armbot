import { NgModule } from '@angular/core';

import {ShellComponent} from "./shell.component";
import {RouterModule} from "@angular/router";
import {ViewService} from "../../../serviсes/view.service";

@NgModule({
  declarations: [
    ShellComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: ShellComponent
    }])
  ],
  exports: [
    ShellComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [ShellComponent]
})
export class ShellModule { }
