import {Component, OnInit} from '@angular/core';
import {StorageService} from "../../serviсes/storage.service";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";

@UntilDestroy()
@Component({
  selector: 'app-user-interface',
  templateUrl: './user-interface.component.html',
  styleUrls: ['./user-interface.component.less']
})
export class UserInterfaceComponent implements OnInit {

  tab: string = 'user-interface';

  constructor(private storageService: StorageService) {

  }

  ngOnInit(): void {
    this.storageService.getUserInterfaceActiveTab().pipe(untilDestroyed(this)).subscribe(data => {
      if (data !== '') {
        this.tab = data;
      }
    });

    this.tab = this.storageService.getUserInterfaceTab();
  }

  changeTab(tab: string) {
    this.storageService.setUserInterfaceActiveTab(tab);
  }

}
