import { NgModule } from '@angular/core';

import {ArduinoComponent} from "./arduino.component";
import {ViewService} from "../../../../serviсes/view.service";

@NgModule({
  declarations: [
    ArduinoComponent
  ],
  imports: [],
  exports: [
    ArduinoComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [ArduinoComponent]
})
export class ArduinoModule { }
