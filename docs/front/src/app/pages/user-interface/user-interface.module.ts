import { NgModule } from '@angular/core';
import {UserInterfaceComponent} from "./user-interface.component";
import {RouterModule, Routes} from "@angular/router";
import {CommonModule} from "@angular/common";
import {DragDirective} from "./dragDrop.directive";
import {MatButtonModule} from "@angular/material/button";
import {HttpClient} from "@angular/common/http";

const routes: Routes = [
      { path: '', component: UserInterfaceComponent },
      { path: '**', loadChildren: () => import('../../components/not-found/not-found.module').then(m => m.NotFoundModule) }
];

@NgModule({
  declarations: [
    UserInterfaceComponent,
    DragDirective
  ],
  imports: [
    RouterModule.forChild(routes),
    CommonModule,
    MatButtonModule
  ],
  providers: [HttpClient],
  bootstrap: [UserInterfaceComponent]
})
export class UserInterfaceModule { }