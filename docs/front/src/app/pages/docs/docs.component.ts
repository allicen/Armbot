import {Component, ViewEncapsulation} from '@angular/core';

@Component({
  selector: 'app-tf',
  templateUrl: './docs.component.html',
  styleUrls: ['./docs.component.less'],
  encapsulation: ViewEncapsulation.None
})
export class DocsComponent {

  constructor() {

  }

  userInterfaceOn: boolean = false;

  selectors:  {[id: string]: string} = {
    'errors': 'Решение проблем',
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
