import {Component, OnInit, ViewEncapsulation} from '@angular/core';
import {ViewService} from "../../serviсes/view.service";

@Component({
  selector: 'app-tf',
  templateUrl: './docs.component.html',
  styleUrls: ['./docs.component.less'],
  encapsulation: ViewEncapsulation.None
})
export class DocsComponent implements OnInit {
  constructor(private viewService: ViewService) {}

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

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
