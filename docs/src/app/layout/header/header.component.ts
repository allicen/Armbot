import {Component, Input} from '@angular/core';
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
    userInterfaceSub: Subscription;

    constructor(public storage: StorageService, private router: Router) {
        this.userInterfaceSub = this.storage.userInterfaceOn$.subscribe(data => this.userInterfaceOn = data);
    }

    userInterfaceChange() {
        this.storage.setData(this.userInterfaceOn);

        if (this.userInterfaceOn) {
            this.router.navigate(['user-interface']);
        } else {
            this.router.navigate(['']);
        }
    }

    ngOnDestroy(): void {
        this.userInterfaceSub.unsubscribe();
    }
}
