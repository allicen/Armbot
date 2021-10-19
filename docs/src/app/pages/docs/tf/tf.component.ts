import {Component, OnInit, ViewEncapsulation} from '@angular/core';
import {ViewService} from "../../../serviсes/view.service";

@Component({
  selector: 'app-tf',
  templateUrl: './tf.component.html',
  styleUrls: ['./tf.component.less', '../../../layout/main/main.component.less'],
  encapsulation: ViewEncapsulation.None
})
export class TfComponent implements OnInit {
  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
