import {Component, OnInit} from '@angular/core';
import {ViewService} from "../../serviсes/view.service";

@Component({
  selector: 'app-gazebo',
  templateUrl: './shell.component.html',
  styleUrls: ['./shell.component.less']
})
export class ShellComponent implements OnInit {
  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
