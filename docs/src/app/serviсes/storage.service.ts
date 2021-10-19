import { Injectable } from '@angular/core';
import { BehaviorSubject, Observable } from "rxjs";
import { Router } from "@angular/router";

@Injectable({ providedIn: 'root' })
export class StorageService {

    public booleanSubject = new BehaviorSubject<boolean>(false);
    userInterfaceOn$: Observable<boolean> = this.booleanSubject.asObservable();

    urlArr = this.router.url.split('/');

    constructor(private router: Router) {
        this.setDataFromUrl();
    }

    setData(userInterfaceOn: boolean) {
        this.booleanSubject.next(userInterfaceOn);
    }

    setDataFromUrl() {
      this.setData(this.urlArr.length > 0 && this.urlArr[1] === 'user-interface');
    }
}
