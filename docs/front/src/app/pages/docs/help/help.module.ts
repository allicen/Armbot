import { NgModule } from '@angular/core';

import {HelpComponent} from "./help.component";
import {RouterModule} from "@angular/router";
import {ViewService} from "../../../serviсes/view.service";
import {RosModule} from "../help-components/ros/ros.module";
import {MoveitModule} from "../help-components/moveit/moveit.module";
import {TfModule} from "../help-components/tf/tf.module";
import {LaunchModule} from "../help-components/launch/launch.module";
import {ArduinoModule} from "../help-components/arduino/arduino.module";
import {DockerModule} from "../help-components/docker/docker.module";
import {ShellModule} from "../help-components/shell/shell.module";
import {LinuxModule} from "../help-components/linux/linux.module";
import {MoreModule} from "../help-components/more/more.module";
import {GazeboModule} from "../help-components/gazebo/gazebo.module";

@NgModule({
  declarations: [
    HelpComponent
  ],
  imports: [
    RouterModule.forChild([{
      path: '',
      component: HelpComponent
    }]),
    RosModule,
    MoveitModule,
    TfModule,
    LaunchModule,
    ArduinoModule,
    DockerModule,
    ShellModule,
    LinuxModule,
    MoreModule,
    GazeboModule
  ],
  exports: [
    HelpComponent
  ],
  providers: [
    ViewService
  ],
  bootstrap: [HelpComponent]
})
export class HelpModule { }
