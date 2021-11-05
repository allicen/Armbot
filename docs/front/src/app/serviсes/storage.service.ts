import { Injectable } from '@angular/core';
import { BehaviorSubject, Observable } from "rxjs";
import { Router } from "@angular/router";
import {HttpService} from "./http.service";
import {Coordinate} from "../model/models";

@Injectable({ providedIn: 'root' })
export class StorageService {

    private userInterfaceOn$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false);
    private coordinateDelete$: BehaviorSubject<number> = new BehaviorSubject<number>(-1);
    private coordinateDeleteError$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false);
    private coordinateDeleteMessage$: BehaviorSubject<string> = new BehaviorSubject<string>('');
    private clickCoordinate$: BehaviorSubject<Coordinate> = new BehaviorSubject<Coordinate>({x: 0, y: 0, z: 0, name: '', id: -1});
    private userInterfaceActiveTab$: BehaviorSubject<string> = new BehaviorSubject<string>('');

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
      this.httpService.removeCoordinate(id).pipe().subscribe((res: any) => {
        if (!res) {
          return;
        }
        this.coordinateDelete$.next(id);
        this.coordinateDeleteError$.next(res.status === 'ERROR');

        let message = '';

        if (res.message) {
          message = res.message;
        }

        this.coordinateDeleteMessage$.next(message);
      })
    }

    getCoordinateDelete(): Observable<number> {
      return this.coordinateDelete$.asObservable();
    }

    getCoordinateDeleteError(): Observable<boolean> {
      return this.coordinateDeleteError$.asObservable();
    }

    getCoordinateDeleteMessage(): Observable<string> {
      return this.coordinateDeleteMessage$.asObservable();
    }

    getClickCoordinate(): Observable<Coordinate> {
      return this.clickCoordinate$.asObservable();
    }

    setClickCoordinate(coordinate: Coordinate): void {
      this.clickCoordinate$.next(coordinate);
    }

    getUserInterfaceActiveTab(): Observable<string> {
      return this.userInterfaceActiveTab$.asObservable();
    }

    setUserInterfaceActiveTab(tab: string): void {
      this.userInterfaceActiveTab$.next(tab);
    }
}
