import {
  Component,
  OnInit,
  ViewEncapsulation
} from '@angular/core';
import {ViewService} from "../../serviсes/view.service";

@Component({
  selector: 'app-docker',
  templateUrl: './docker.component.html',
  styleUrls: ['../../layout/main/main.component.less', './docker.component.less'],
  encapsulation: ViewEncapsulation.None
})
export class DockerComponent implements OnInit {
  copyText: string = "Скопировано!";

  constructor(private viewService: ViewService) {}

  ngOnInit(): void {
    this.viewService.viewToolTip();
  }
}
