import {Component, Input, Output} from '@angular/core';
import {StorageService} from "../../serviсes/storage.service";
import {Subscription} from "rxjs";
import {Router} from "@angular/router";

@Component({
    selector: 'app-header',
    templateUrl: './header.component.html',
    styleUrls: ['./header.component.less']
})
export class HeaderComponent {
    title = 'user-interface';

    userInterfaceOn: boolean = false;

    constructor(public storage: StorageService, private router: Router) {
        this.storage.getUserInterface().subscribe(data => this.userInterfaceOn = data);
    }

    userInterfaceChange() {

        if (this.userInterfaceOn) {
            this.router.navigate(['user-interface']);
        } else {
            this.router.navigate(['']);
        }
      this.storage.setUserInterface(this.userInterfaceOn);
    }
}
