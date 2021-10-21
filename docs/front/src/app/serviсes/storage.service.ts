import { Injectable } from '@angular/core';
import { BehaviorSubject, Observable } from "rxjs";
import { Router } from "@angular/router";

@Injectable({ providedIn: 'root' })
export class StorageService {

    private userInterfaceOn$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false);
    urlArr = this.router.url.split('/');

    isUserInterface(): boolean {
      return this.urlArr.length > 0 && this.urlArr[1] === 'user-interface';
    }

    constructor(private router: Router) {
        router.events.subscribe(() => {
            this.setUserInterface(this.isUserInterface());
        });
    }

    setUserInterface(showHide: boolean): void {
        this.userInterfaceOn$.next(showHide);
    }

    getUserInterface(): Observable<boolean> {
        return this.userInterfaceOn$.asObservable();
    }
}
