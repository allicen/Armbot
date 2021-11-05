import {Component, OnInit} from '@angular/core';
import {StorageService} from "../../serviсes/storage.service";

@Component({
  selector: 'app-user-interface',
  templateUrl: './user-interface.component.html',
  styleUrls: ['./user-interface.component.less']
})
export class UserInterfaceComponent implements OnInit {

  tab: string = 'command-lib';

  constructor(private storageService: StorageService) {

  }

  ngOnInit(): void {
    this.storageService.getUserInterfaceActiveTab().subscribe(data => {
      if (data !== '') {
        this.tab = data;
      }
    })
  }

  changeTab(tab: string) {
    this.storageService.setUserInterfaceActiveTab(tab);
  }

}
