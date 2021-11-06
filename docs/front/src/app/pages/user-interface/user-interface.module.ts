import { NgModule } from '@angular/core';
import {UserInterfaceComponent} from "./user-interface.component";
import {RouterModule, Routes} from "@angular/router";
import {MatTabsModule} from "@angular/material/tabs";
import {OpenDialogComponent} from "./open-dialog/open-dialog.component";
import {MatDialogModule} from "@angular/material/dialog";
import {MatButtonModule} from "@angular/material/button";
import {CommonModule} from "@angular/common";


const routes: Routes = [
  { path: '', component: UserInterfaceComponent,
    children: [
      { path: '', loadChildren: () => import('../../pages/user-interface/command-lib/command-lib.module').then(m => m.CommandLibModule) },
      { path: 'generate-file', loadChildren: () => import('../../pages/user-interface/generate-file/generate-file.module').then(m => m.GenerateFileModule) },
      { path: '**', loadChildren: () => import('../../components/not-found/not-found.module').then(m => m.NotFoundModule) }
    ]}
];



@NgModule({
  declarations: [
    UserInterfaceComponent,
    OpenDialogComponent
  ],
    imports: [
        RouterModule.forChild(routes),
        MatTabsModule,
        MatDialogModule,
        MatButtonModule,
        CommonModule
    ],
  providers: [],
  bootstrap: [UserInterfaceComponent]
})
export class UserInterfaceModule { }
