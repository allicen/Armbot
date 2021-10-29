import { Injectable } from '@angular/core';
import { BehaviorSubject, Observable } from "rxjs";
import { Router } from "@angular/router";
import {HttpService} from "./http.service";
import {Response} from "../model/models";

@Injectable({ providedIn: 'root' })
export class StorageService {

    private userInterfaceOn$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false);
    private coordinateDelete$: BehaviorSubject<number> = new BehaviorSubject<number>(-1);
    urlArr = this.router.url.split('/');

    isUserInterface(): boolean {
      return this.urlArr.length > 0 && this.urlArr[1] === 'user-interface';
    }

    constructor(private router: Router, private httpService: HttpService) {
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

    setCoordinateDelete(id: number): void {
      this.httpService.removeCoordinate(id).pipe().subscribe((res) => {
        if (!res) {
          return;
        }
        this.coordinateDelete$.next(id);
      })
    }

    getCoordinateDelete(): Observable<number> {
      return this.coordinateDelete$.asObservable();
    }
}
