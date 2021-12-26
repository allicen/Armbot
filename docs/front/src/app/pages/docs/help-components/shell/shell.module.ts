import { NgModule } from '@angular/core';

import {ShellComponent} from "./shell.component";

@NgModule({
  declarations: [
    ShellComponent
  ],
  imports: [],
  exports: [
    ShellComponent
  ],
  providers: [],
  bootstrap: [ShellComponent]
})
export class ShellModule { }
