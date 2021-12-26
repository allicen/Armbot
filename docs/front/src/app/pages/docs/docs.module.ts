import { NgModule } from '@angular/core';

import {DocsComponent} from "./docs.component";
import {RouterModule, Routes} from "@angular/router";
import {CommonModule} from "@angular/common";
import {MatButtonToggleModule} from "@angular/material/button-toggle";;

const routes: Routes = [
    { path: '', component: DocsComponent,
        children: [
            { path: '', loadChildren: () => import('./general/general.module').then(m => m.GeneralModule) },
            { path: 'help', loadChildren: () => import('./help/help.module').then(m => m.HelpModule) },
            { path: 'help', loadChildren: () => import('./help/help.module').then(m => m.HelpModule)},
            { path: 'errors', loadChildren: () => import('./errors/errors.module').then(m => m.ErrorsModule) },
            { path: '**', loadChildren: () => import('../../components/not-found/not-found.module').then(m => m.NotFoundModule) }
        ]}
];

@NgModule({
  declarations: [
    DocsComponent
  ],
  imports: [
    RouterModule.forChild(routes),
    MatButtonToggleModule,
    CommonModule
  ],
  exports: [RouterModule, DocsComponent],
  providers: [],
  bootstrap: [DocsComponent]
})
export class DocsModule { }
