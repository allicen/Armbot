import {Component, OnInit, ViewEncapsulation} from '@angular/core';
import {ViewService} from "../../serviсes/view.service";

@Component({
  selector: 'app-moveit',
  templateUrl: './moveit.component.html',
  styleUrls: ['./moveit.component.less', '../../layout/main/main.component.less'],
  encapsulation: ViewEncapsulation.None
})
export class MoveitComponent implements OnInit {
  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
