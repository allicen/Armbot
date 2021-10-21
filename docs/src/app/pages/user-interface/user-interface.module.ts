import { NgModule } from '@angular/core';
import {UserInterfaceComponent} from "./user-interface.component";
import {RouterModule, Routes} from "@angular/router";

const routes: Routes = [
      { path: '', component: UserInterfaceComponent },
      { path: '**', loadChildren: () => import('../../components/not-found/not-found.module').then(m => m.NotFoundModule) }
];

@NgModule({
  declarations: [
    UserInterfaceComponent
  ],
  imports: [
    RouterModule.forChild(routes)
  ],
  providers: [],
  bootstrap: [UserInterfaceComponent]
})
export class UserInterfaceModule { }
