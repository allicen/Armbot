import { NgModule } from '@angular/core';
import {UserInterfaceComponent} from "./user-interface.component";
import {RouterModule, Routes} from "@angular/router";

const routes: Routes = [
      { path: '', component: UserInterfaceComponent },
      { path: 'upload-file', loadChildren: () => import('../../pages/user-interface/upload-file/upload-file.module').then(m => m.UploadFileModule) },
      { path: 'download-file', loadChildren: () => import('../../pages/user-interface/download-file/download-file.module').then(m => m.DownloadFileModule) },
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
