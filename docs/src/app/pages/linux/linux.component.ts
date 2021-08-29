import {Component, OnInit, ViewEncapsulation} from '@angular/core';
import {ViewService} from "../../serviсes/view.service";

@Component({
  selector: 'app-linux',
  templateUrl: './linux.component.html',
  styleUrls: ['./linux.component.less', '../../layout/main/main.component.less'],
  encapsulation: ViewEncapsulation.None
})
export class LinuxComponent implements OnInit {
  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
