import { NgModule } from '@angular/core';

import {RouterModule, Routes} from "@angular/router";
import {MainModule} from "./layout/main/main.module";

const appRoutes: Routes = [
  { path: '', loadChildren: () => MainModule }
];

@NgModule({
  imports: [
    RouterModule.forRoot(appRoutes)
  ],
  exports: [RouterModule]
})
export class AppRoutingModule { }
