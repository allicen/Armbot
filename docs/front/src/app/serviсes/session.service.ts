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
    private nextId$: BehaviorSubject<number> = new BehaviorSubject<number>(0);
    private diameterPoint$: BehaviorSubject<number> = new BehaviorSubject<number>(6);

    private maxId: number = 0;
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
            this.addImageDetails(data.details.image);
        }

        if (data.status === 'SUCCESS' && data.details.coordinateList) {
            this.addCoordinateList(data.details.coordinateList);
        }

        if (data.status === 'SUCCESS' && data.details.launchFileRowList) {
            this.addFileRows(data.details.launchFileRowList);
        }
    }

    addImageDetails(image: any) {
        this.image$.next(this.sanitizer.bypassSecurityTrustResourceUrl(`data:${image.contentType};base64,${image.imageByte}`));
        this.imagePosition$.next({x: image.imagePositionX, y: image.imagePositionY});
        this.imageWidth$.next(image.imageWidthPx);
        this.canEditImage$.next(image.canEdit);

        if (image.canEdit) {
            this.storageService.setCurrentStep(2);
        } else {
            this.storageService.setCurrentStep(3);
        }
    }

    addCoordinateList(coordinates: any) {
        const list: Coordinate[] = [];
        coordinates.forEach((item: any) => {
            const coordinate: Coordinate = {id: item.id, name: item.name, x: item.x, y: item.y, z: item.z};
            list.push(coordinate);
        });

        this.setCoordinateList(list);
    }

    addFileRows(fileRows: any) {
        const list: LaunchFileRow[] = [];
        let maxId: number = 0;
        fileRows.forEach((item: any) => {
            const coordinate: Coordinate | undefined = this.coordinateList.find(c => c.id == item.coordinateId);
            if (coordinate) {
                const fileRow: LaunchFileRow = {coordinate: coordinate, id: item.id, delay: item.delay, sortOrder: item.sortOrder};
                list.push(fileRow);

                if (maxId < item.id) {
                    maxId = item.id;
                }
            }
        });

        this.setNextFileRowId(maxId);
        this.setLaunchFileRow(list);
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
                this.nextId$.next(0);

                this.storageService.clearVariable();
                this.messageService.clearVariable();

                this.maxId = 0;
                this.coordinateList = [];
                this.launchFileRowList = [];
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
        this.launchFileRowList = launchFileRowList;
        this.launchFileRow$.next(launchFileRowList);
    }

    addLaunchFileRowList(launchFileRow: LaunchFileRow): void {
        this.httpService.saveLaunchFileRow(launchFileRow.coordinate.id, launchFileRow.delay).pipe(untilDestroyed(this)).subscribe(data => {
            if (data && data.status === 'SUCCESS') {
                this.launchFileRowList.push(launchFileRow);
                this.setLaunchFileRow(this.launchFileRowList);
                this.setNextFileRowId(this.maxId + 1);
                this.storageService.setClickCoordinate(launchFileRow.coordinate);
            }
        });
    }

    changeDelay(itemId: number, delay: string): void {
        this.httpService.updateDelayFileRow(itemId, Number(delay)).pipe().subscribe(data => {
            if (data.status === 'SUCCESS') {
                const rowIndex: number = this.launchFileRowList.findIndex(c => c.id == itemId);
                this.launchFileRowList[rowIndex].delay = Number(delay);
                this.setLaunchFileRow(this.launchFileRowList);
            }
        });
    }

    launchFileRowListSort(fileRows: LaunchFileRow[]) {
        const sortOrderIndex: number[] = [];
        this.launchFileRowList.forEach(item => sortOrderIndex.push(item.id));
        this.httpService.updateSortOrderFileRow(sortOrderIndex.join(','))
            .pipe().subscribe(data => {
                if (data.status !== 'SUCCESS') {
                    this.setLaunchFileRow(fileRows);
                }
        });
    }

    getNextFileRowId(): Observable<number> {
        return this.nextId$.asObservable();
    }

    setNextFileRowId(id: number): void {
        this.maxId = id;
        this.nextId$.next(id);
    }

    importSession(file: File) {
        this.httpService.importSessionJson(file).pipe(untilDestroyed(this)).subscribe(res => {
            if (!res) {
                return;
            }

            if (res.status === 'SUCCESS') {
                this.uploadSession();
            } else {
                this.messageService.setMessageImportErrors(res.details || []);
            }

            this.messageService.setMessageImport(res.message);
        });
    }

    getDiameterPoint(): Observable<number> {
        return this.diameterPoint$.asObservable();
    }

    setDiameterPoint(diameter: number): void {
        this.diameterPoint$.next(diameter);
    }
}
