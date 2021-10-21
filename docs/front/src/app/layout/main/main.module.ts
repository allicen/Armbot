import { NgModule } from '@angular/core';

import {MainComponent} from "./main.component";
import {RouterModule, Routes} from "@angular/router";
import {CommonModule} from "@angular/common";
import {HeaderModule} from "../header/header.module";
import {FooterModule} from "../footer/footer.module";
import {MatGridListModule} from "@angular/material/grid-list";
import {MatListModule} from "@angular/material/list";
import {MatButtonToggleModule} from "@angular/material/button-toggle";
import {MatFormFieldModule} from "@angular/material/form-field";
import {MatIconModule} from "@angular/material/icon";
import {MatInputModule} from "@angular/material/input";

const routes: Routes = [
  { path: '', component: MainComponent,
    children: [
      { path: 'user-interface', loadChildren: () => import('../../pages/user-interface/user-interface.module').then(m => m.UserInterfaceModule) },
      { path: '', loadChildren: () => import('../../pages/docs/docs.module').then(m => m.DocsModule) },
      { path: '**', loadChildren: () => import('../../components/not-found/not-found.module').then(m => m.NotFoundModule) }
      ]
  }];

@NgModule({
  declarations: [
    MainComponent
  ],
  imports: [
    CommonModule,
    RouterModule.forChild(routes),
    HeaderModule,
    FooterModule,
    MatGridListModule,
    MatListModule,
    MatButtonToggleModule,
    MatFormFieldModule,
    MatIconModule,
    MatInputModule
  ],
  bootstrap: [MainComponent]
})
export class MainModule { }
