import {Injectable} from "@angular/core";
import {BehaviorSubject, Observable} from "rxjs";
import {HttpService} from "./http.service";
import {DomSanitizer} from "@angular/platform-browser";
import {Coordinate, LaunchFileRow, Position} from "../model/models";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {StorageService} from "./storage.service";
import {MessageService} from "./message.service";

@UntilDestroy()
@Injectable({ providedIn: 'root' })
export class SessionService {
    constructor(private httpService: HttpService,
                private sanitizer: DomSanitizer,
                private storageService: StorageService,
                private messageService: MessageService) {
    }

    private sessionExists$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false);
    private image$: BehaviorSubject<any> = new BehaviorSubject<any>(null);
    private imagePosition$: BehaviorSubject<Position> = new BehaviorSubject<Position>({x: 0, y: 0});
    private imageWidth$: BehaviorSubject<number> = new BehaviorSubject<number>(0);
    private canEditImage$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(true);
    private coordinateList$: BehaviorSubject<Coordinate[]> = new BehaviorSubject<Coordinate[]>([]);
    private workOptionKey$: BehaviorSubject<string> = new BehaviorSubject<string>('uploadImage');
    private launchFileRow$: BehaviorSubject<LaunchFileRow[]> = new BehaviorSubject<LaunchFileRow[]>([]);

    private coordinateList: Coordinate[] = [];
    private launchFileRowList: LaunchFileRow[] = [];


    getSession(): Observable<any> {
        this.uploadSession();
        return this.sessionExists$.asObservable();
    }

    getSessionExists(): Observable<boolean> {
        return this.sessionExists$.asObservable();
    }

    setSession(sessionExist: boolean): void {
        this.sessionExists$.next(sessionExist);
    }

    getImage(): Observable<any> {
        return this.image$.asObservable();
    }

    uploadSession(): void {
        this.httpService.getSession()
            .pipe(untilDestroyed(this))
            .subscribe(data => {
                this.fillFieldsSession(data);
        });
    }

    fillFieldsSession(data: any) {

        if (data.status === 'NO_SESSION') {
            this.sessionExists$.next(false);
            return;
        }

        if (data.status === 'SUCCESS') {
            this.sessionExists$.next(true);
            this.workOptionKey$.next(data.details.workOption);
        }

        if (data.status === 'SUCCESS' && data.details.image) {
            this.image$.next(this.sanitizer.bypassSecurityTrustResourceUrl(`data:${data.details.image.contentType};base64,${data.details.image.imageByte}`));
            this.imagePosition$.next({x: data.details.image.imagePositionX, y: data.details.image.imagePositionY});
            this.imageWidth$.next(data.details.image.imageWidthPx);
            this.canEditImage$.next(data.details.image.canEdit);

            if (data.details.image.canEdit) {
                this.storageService.setCurrentStep(2);
            } else {
                this.storageService.setCurrentStep(3);
            }
        }

        if (data.status === 'SUCCESS' && data.details.coordinateList) {
            const list: Coordinate[] = [];
            data.details.coordinateList.forEach((item: any) => {
                const coordinate: Coordinate = {id: item.id, name: item.name, x: item.x, y: item.y, z: item.z};
                list.push(coordinate);
            });

            this.setCoordinateList(list);
        }
    }

    removeSession(): void {
        this.httpService.removeSession().pipe(untilDestroyed(this)).subscribe(data => {
            if (data.status === 'SUCCESS') {

                // очищаем все поля
                this.sessionExists$.next(false);
                this.image$.next(null);
                this.imagePosition$.next({x: 0, y: 0});
                this.imageWidth$.next(0);
                this.canEditImage$.next(true);
                this.coordinateList$.next([]);
                this.workOptionKey$.next('uploadImage');

                this.storageService.clearVariable();
                this.messageService.clearVariable();
            }
        });
    }

    getImagePosition(): Observable<Position> {
        return this.imagePosition$.asObservable();
    }

    setImagePosition(x: number, y: number): void {
        this.imagePosition$.next({x: x, y: y});
    }

    getImageEditAllowed(): Observable<boolean> {
        return this.canEditImage$.asObservable();
    }

    setImageEditAllowed(allowed: boolean): void {
        this.canEditImage$.next(allowed);
    }

    getImageWidth(): Observable<number> {
        return this.imageWidth$.asObservable();
    }

    setImageWidth(width: number): void {
        this.imageWidth$.next(width);
    }

    setCoordinateList(coordinateList: Coordinate[]): void {
        this.coordinateList = coordinateList;
        this.coordinateList$.next(this.coordinateList);
    }

    getCoordinateList(): Observable<Coordinate[]> {
        return this.coordinateList$.asObservable();
    }

    addCoordinateInList(coordinate: Coordinate): void {
         this.coordinateList.push(coordinate);
         this.setCoordinateList(this.coordinateList);
    }

    getWorkOptionKey(): Observable<string> {
        return this.workOptionKey$.asObservable();
    }

    setWorkOptionKey(option: string): void {
        this.workOptionKey$.next(option);
    }

    getLaunchFileRow(): Observable<LaunchFileRow[]> {
        return this.launchFileRow$.asObservable();
    }

    setLaunchFileRow(launchFileRowList: LaunchFileRow[]): void {
        this.launchFileRow$.next(launchFileRowList);
    }

    addLaunchFileRowList(launchFileRow: LaunchFileRow): void {
        this.launchFileRowList.push(launchFileRow);
        this.setLaunchFileRow(this.launchFileRowList);
    }

    changeDelay(itemId: number, delay: string): void {
        const rowIndex: number = this.launchFileRowList.findIndex(c => c.id == itemId);
        this.launchFileRowList[rowIndex].delay = Number(delay);
        this.setLaunchFileRow(this.launchFileRowList);
    }

    launchFileRowListSort(firstElemId: number, secondElemId: number, firstSortOrder: number, secondSortOrder: number) {
        const firstIndex: number = this.launchFileRowList.findIndex(c => c.id === firstElemId);
        const secondIndex: number = this.launchFileRowList.findIndex(c => c.id === secondElemId);

        this.launchFileRowList[firstIndex].sortOrder = secondSortOrder;
        this.launchFileRowList[secondIndex].sortOrder = firstSortOrder;

        this.setLaunchFileRow(this.launchFileRowList);
    }

}
