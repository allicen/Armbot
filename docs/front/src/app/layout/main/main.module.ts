import { NgModule } from '@angular/core';

import {MainComponent} from "./main.component";
import {RouterModule, Routes} from "@angular/router";
import {HeaderModule} from "../header/header.module";
import {FooterModule} from "../footer/footer.module";
import {CommonModule} from "@angular/common";

const routes: Routes = [
  { path: '', component: MainComponent,
    children: [
      { path: '', pathMatch: 'full', redirectTo: 'docs' },
      { path: 'docs', loadChildren: () => import('../../pages/docs/docs.module').then(m => m.DocsModule) },
      { path: 'user-interface', loadChildren: () => import('../../pages/user-interface/user-interface.module').then(m => m.UserInterfaceModule) },
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
    FooterModule
  ],
  exports: [RouterModule],
  bootstrap: [MainComponent]
})
export class MainModule { }
