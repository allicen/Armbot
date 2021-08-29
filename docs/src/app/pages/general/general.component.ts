import {Component, OnInit} from '@angular/core';
import {ViewService} from "../../serviсes/view.service";

@Component({
  selector: 'app-gazebo',
  templateUrl: './general.component.html',
  styleUrls: ['./general.component.less']
})
export class GeneralComponent implements OnInit {
  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
