import {Component, OnInit} from '@angular/core';
import {StorageService} from "../../serviсes/storage.service";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {Router} from "@angular/router";
import {SessionService} from "../../serviсes/session.service";

@UntilDestroy()
@Component({
  selector: 'app-user-interface',
  templateUrl: './user-interface.component.html',
  styleUrls: ['./user-interface.component.less']
})
export class UserInterfaceComponent implements OnInit {

  tab: string = 'user-interface';
  sessionExists: boolean = false;

  constructor(private storageService: StorageService, private router: Router, private sessionService: SessionService) {
  }

  ngOnInit(): void {
    this.storageService.getUserInterfaceActiveTab().pipe(untilDestroyed(this)).subscribe(data => {
      if (data !== '') {
        this.tab = data;
      }
    });

      this.tab = this.storageService.getUserInterfaceTab(this.router.url);

      this.sessionService.getSession().pipe(untilDestroyed(this)).subscribe(data => {
      this.sessionExists = data;
    });
  }

  changeTab(tab: string) {
    this.storageService.setUserInterfaceActiveTab(tab);
  }

}
