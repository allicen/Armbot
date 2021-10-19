import {Component, OnInit, ViewEncapsulation} from '@angular/core';
import {ViewService} from "../../../serviсes/view.service";

@Component({
  selector: 'app-gazebo',
  templateUrl: './arduino.component.html',
  styleUrls: ['./arduino.component.less', '../../../layout/main/main.component.less'],
  encapsulation: ViewEncapsulation.None
})
export class ArduinoComponent implements OnInit {
  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
