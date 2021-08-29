import {Component, OnInit, ViewEncapsulation} from '@angular/core';
import {ViewService} from "../../serviсes/view.service";

@Component({
  selector: 'app-more',
  templateUrl: './more.component.html',
  styleUrls: ['./more.component.less', '../../layout/main/main.component.less'],
  encapsulation: ViewEncapsulation.None
})
export class MoreComponent implements OnInit {
  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
