import {Component, OnInit} from '@angular/core';
import {ViewService} from "../../serviсes/view.service";

@Component({
  selector: 'app-gazebo',
  templateUrl: './moveit.component.html',
  styleUrls: ['./moveit.component.less']
})
export class MoveitComponent implements OnInit {
  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
