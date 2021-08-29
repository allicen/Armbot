import {Component, OnInit, ViewEncapsulation} from '@angular/core';
import {ViewService} from "../../serviсes/view.service";

@Component({
  selector: 'app-gazebo',
  templateUrl: './gazebo.component.html',
  styleUrls: ['./gazebo.component.less', '../../layout/main/main.component.less'],
  encapsulation: ViewEncapsulation.None
})
export class GazeboComponent implements OnInit {
  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
