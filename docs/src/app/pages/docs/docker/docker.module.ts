import { NgModule } from '@angular/core';

import {DockerComponent} from "./docker.component";
import {RouterModule} from "@angular/router";
import {MatTooltipModule} from "@angular/material/tooltip";
import {ViewService} from "../../../serviсes/view.service";

@NgModule({
  declarations: [
    DockerComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: DockerComponent
    }]),
    MatTooltipModule
  ],
  exports: [
    DockerComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [DockerComponent]
})
export class DockerModule { }
