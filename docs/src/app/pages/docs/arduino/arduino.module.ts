import { NgModule } from '@angular/core';

import {ArduinoComponent} from "./arduino.component";
import {RouterModule} from "@angular/router";
import {ViewService} from "../../../serviсes/view.service";

@NgModule({
  declarations: [
    ArduinoComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: ArduinoComponent
    }])
  ],
  exports: [
    ArduinoComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [ArduinoComponent]
})
export class ArduinoModule { }
