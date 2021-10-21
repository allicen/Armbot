import {Component, OnInit, ViewEncapsulation} from '@angular/core';
import {ViewService} from "../../../serviсes/view.service";

@Component({
  selector: 'app-launch',
  templateUrl: './launch.component.html',
  styleUrls: ['./launch.component.less', '../../../layout/main/main.component.less'],
  encapsulation: ViewEncapsulation.None
})
export class LaunchComponent implements OnInit {
  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
