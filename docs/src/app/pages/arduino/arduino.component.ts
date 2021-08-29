import {Component, OnInit} from '@angular/core';
import {ViewService} from "../../serviсes/view.service";

@Component({
  selector: 'app-gazebo',
  templateUrl: './arduino.component.html',
  styleUrls: ['./arduino.component.less']
})
export class ArduinoComponent implements OnInit {
  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
