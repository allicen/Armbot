import {Component, ElementRef, Input, Output, ViewChild} from '@angular/core';
import {StorageService} from "../../serviсes/storage.service";
import {Subscription} from "rxjs";
import {Router} from "@angular/router";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {MatSlideToggle, MatSlideToggleChange} from "@angular/material/slide-toggle";

@UntilDestroy()
@Component({
    selector: 'app-header',
    templateUrl: './header.component.html',
    styleUrls: ['./header.component.less']
})
export class HeaderComponent {
    title = 'user-interface';

    userInterfaceOn: boolean = false;

    constructor(public storage: StorageService, private router: Router) {
        this.storage.getUserInterface().pipe(untilDestroyed(this)).subscribe(data => this.userInterfaceOn = data);
    }

    userInterfaceChange($event: MatSlideToggleChange) {
        this.userInterfaceOn = $event.checked;
        this.navigate(this.userInterfaceOn);
    }

    goToHome() {
        this.navigate(false);
        this.storage.changeInterface();
    }


    navigate(on: boolean) {
        if (on) {
            this.router.navigate(['user-interface']);
        } else {
            this.router.navigate(['docs']);
        }

        this.storage.setUserInterface(this.userInterfaceOn);
    }
}
