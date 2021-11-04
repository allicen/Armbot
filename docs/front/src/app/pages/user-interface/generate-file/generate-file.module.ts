import { NgModule } from '@angular/core';
import {GenerateFileComponent} from "./generate-file.component";
import {RouterModule} from "@angular/router";

@NgModule({
  declarations: [
    GenerateFileComponent,
  ],
    imports: [
        RouterModule.forChild([{
          path: '',
          component: GenerateFileComponent
        }]),
    ],
  exports: [GenerateFileComponent],
  providers: [],
  bootstrap: [GenerateFileComponent]
})
export class GenerateFileModule { }
