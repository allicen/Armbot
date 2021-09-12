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
      { path: '', loadChildren: () => import('../../pages/general/general.module').then(m => m.GeneralModule) },
      { path: 'ros', loadChildren: () => import('../../pages/ros/ros.module').then(m => m.RosModule) },
      { path: 'launch', loadChildren: () => import('../../pages/launch/launch.module').then(m => m.LaunchModule) },
      { path: 'gazebo', loadChildren: () => import('../../pages/gazebo/gazebo.module').then(m => m.GazeboModule) },
      { path: 'docker', loadChildren: () => import('../../pages/docker/docker.module').then(m => m.DockerModule) },
      { path: 'shell', loadChildren: () => import('../../pages/shell/shell.module').then(m => m.ShellModule) },
      { path: 'moveit', loadChildren: () => import('../../pages/moveit/moveit.module').then(m => m.MoveitModule) },
      { path: 'arduino', loadChildren: () => import('../../pages/arduino/arduino.module').then(m => m.ArduinoModule) },
      { path: 'tf', loadChildren: () => import('../../pages/tf/tf.module').then(m => m.TfModule) },
      { path: 'more', loadChildren: () => import('../../pages/more/more.module').then(m => m.MoreModule) },
      { path: 'linux', loadChildren: () => import('../../pages/linux/linux.module').then(m => m.LinuxModule) },
      { path: '**', loadChildren: () => import('../../components/not-found/not-found.module').then(m => m.NotFoundModule) }]
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
