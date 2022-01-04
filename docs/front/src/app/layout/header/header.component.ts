import {Component, OnInit} from '@angular/core';
import {StorageService} from "../../serviсes/storage.service";
import {Route, Router} from "@angular/router";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {MatSlideToggleChange} from "@angular/material/slide-toggle";
import {ArmbotService} from "../../serviсes/armbot.service";

@UntilDestroy()
@Component({
    selector: 'app-header',
    templateUrl: './header.component.html',
    styleUrls: ['./header.component.less']
})
export class HeaderComponent implements OnInit {
    title = 'user-interface';

    userInterfaceOn: boolean = false;

    armbotMessage: string = '';
    armbotStatus: string = '';
    armbotOnTurn: boolean = false;
    armbotOnAttempt: number = 0;

    constructor(public storage: StorageService,
                private router: Router,
                private armbotService: ArmbotService) {
    }

    ngOnInit(): void {
        this.storage.getUserInterface().pipe(untilDestroyed(this)).subscribe(data => this.userInterfaceOn = data);
        this.armbotService.getArmbotStatus().pipe(untilDestroyed(this)).subscribe(data => {
            this.armbotStatus = data;
            if (data === 'disconnect') {
                this.armbotMessage = 'Робот отключен';
                this.armbotOnTurn = false;
            } else {
                this.armbotMessage = 'Робот подключен';
                this.armbotOnTurn = true;
                this.armbotOnAttempt = 0;
            }
        });
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

    armbotOn() {
        this.armbotService.armbotOn();
        this.armbotOnAttempt++;
    }
}
