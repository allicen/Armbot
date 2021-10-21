import { NgModule } from '@angular/core';

import {DocsComponent} from "./docs.component";
import {RouterModule, Routes} from "@angular/router";
import {MatButtonToggleModule} from "@angular/material/button-toggle";
import {MatIconModule} from "@angular/material/icon";
import {MatFormFieldModule} from "@angular/material/form-field";
import {ViewService} from "../../serviсes/view.service";
import {CommonModule} from "@angular/common";
import {MainComponent} from "../../layout/main/main.component";
import {MatInputModule} from "@angular/material/input";

const routes: Routes = [
    { path: '', component: DocsComponent,
        children: [
            { path: '', loadChildren: () => import('../../pages/docs/general/general.module').then(m => m.GeneralModule) },
            { path: 'ros', loadChildren: () => import('../../pages/docs/ros/ros.module').then(m => m.RosModule) },
            { path: 'launch', loadChildren: () => import('../../pages/docs/launch/launch.module').then(m => m.LaunchModule) },
            { path: 'gazebo', loadChildren: () => import('../../pages/docs/gazebo/gazebo.module').then(m => m.GazeboModule) },
            { path: 'docker', loadChildren: () => import('../../pages/docs/docker/docker.module').then(m => m.DockerModule) },
            { path: 'shell', loadChildren: () => import('../../pages/docs/shell/shell.module').then(m => m.ShellModule) },
            { path: 'moveit', loadChildren: () => import('../../pages/docs/moveit/moveit.module').then(m => m.MoveitModule) },
            { path: 'arduino', loadChildren: () => import('../../pages/docs/arduino/arduino.module').then(m => m.ArduinoModule) },
            { path: 'tf', loadChildren: () => import('../../pages/docs/tf/tf.module').then(m => m.TfModule) },
            { path: 'more', loadChildren: () => import('../../pages/docs/more/more.module').then(m => m.MoreModule) },
            { path: 'linux', loadChildren: () => import('../../pages/docs/linux/linux.module').then(m => m.LinuxModule) },
            { path: 'errors', loadChildren: () => import('../../pages/docs/errors/errors.module').then(m => m.ErrorsModule) },
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
    MatIconModule,
    MatFormFieldModule,
    CommonModule,
    MatInputModule
  ],
  exports: [
    DocsComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [DocsComponent]
})
export class DocsModule { }
