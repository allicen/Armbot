import { Component } from '@angular/core';
import {MAT_TOOLTIP_DEFAULT_OPTIONS, MatTooltipDefaultOptions} from "@angular/material/tooltip";

export const myCustomTooltipDefaults: MatTooltipDefaultOptions = {
  showDelay: 500,
  hideDelay: 500,
  touchendHideDelay: 500,
};

@Component({
  selector: 'app-main',
  templateUrl: './main.component.html',
  styleUrls: ['./main.component.less'],
  providers: [
    {provide: MAT_TOOLTIP_DEFAULT_OPTIONS, useValue: myCustomTooltipDefaults}
  ],
})

export class MainComponent {
  selectors:  {[id: string]: string} = {
    'ros': 'ROS',
    'docker': 'Docker',
    'shell': 'Shell',
    'launch':'Launch',
    'gazebo':'Gazebo',
    'moveit': 'MoveIt!',
    'arduino': 'Arduino',
    'tf': 'TF',
    'linux': 'Linux',
    'more': 'Прочее'
  };

  objectKeys(selectors: {}) {
    return Object.keys(selectors);
  }
}
