import {Component, OnInit, ViewEncapsulation} from '@angular/core';
import {ViewService} from "../../../serviсes/view.service";

@Component({
  selector: 'app-more',
  templateUrl: './errors.component.html',
  styleUrls: ['./errors.component.less', '../../../layout/main/main.component.less'],
  encapsulation: ViewEncapsulation.None
})
export class ErrorsComponent implements OnInit {
  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
