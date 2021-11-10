import {Injectable} from "@angular/core";
import {EMPTY, Observable, of, Subscription, throwError} from "rxjs";
import {HttpClient, HttpErrorResponse} from "@angular/common/http";
import {FileHandle} from "../pages/user-interface/command-lib/import-image/dragDrop.directive";
import {Config} from "../config/config";
import {catchError, map} from "rxjs/operators";
import {Coordinate, Position} from "../model/models";


@Injectable({ providedIn: 'root' })
export class HttpService {

    constructor(private http: HttpClient, private config: Config){ }

    /**
     * Загрузка изображения клавиатуры
     * */
    uploadImage(file: FileHandle): Observable<any> {

        const formData: FormData = new FormData();
        formData.append('file', file.file);
        formData.append('name', file.file.name);
        formData.append('contentType', file.file.type);

        return this.http.post(`${this.config.httpUrl}/image/upload`, formData).pipe(
            map(res => {
                return res;
            })
        );
    }


    /**
     * Импорт координат из файла
     * */
    importCoordinates(file: File): Observable<any> {

        if (!file) {
          return EMPTY;
        }

        const formData: FormData = new FormData();
        formData.append('file', file);
        formData.append('contentType', file.type);

        return this.http.post(`${this.config.httpUrl}/coordinate/import`, formData).pipe(
            map(res => {
                return res;
            })
        );
    }


    /**
     * Удалить сессию
     * */
    removeSession(): Observable<any> {
        return this.http.get(`${this.config.httpUrl}/session/remove/`).pipe(
            map(res => {
                return res;
            })
        );
    }


    /**
     * Обновить координату
     * */
    saveCoordinate(coordinate: Coordinate): Observable<any> {
        return this.http.post(`${this.config.httpUrl}/coordinate/save`, coordinate).pipe(
            catchError((error: HttpErrorResponse) => {
                return throwError(error);
            }),
            map(res => {
               return res;
            }),
        );
    }


    /**
     * Обновить координату
     * */
    updateCoordinate(coordinate: Coordinate): Observable<any> {
        return this.http.post(`${this.config.httpUrl}/coordinate/update`, coordinate).pipe(
            catchError((error: HttpErrorResponse) => {
                return throwError(error);
            }),
            map(res => {
                return res;
            }),
        );
    }


    /**
     * Удалить все координаты
     * */
    removeAllCoordinates(): Observable<any> {
        return this.http.get(`${this.config.httpUrl}/coordinate/removeAll`).pipe(
            map(res => {
                return res;
            })
        );
    }


    /**
     * Удалить координату
     * */
    removeCoordinate(id: number) {
        return this.http.get(`${this.config.httpUrl}/coordinate/remove/${id}`).pipe(
            catchError((error: HttpErrorResponse) => {
                return throwError(error);
            }),
            map(res => {
                return res;
            }),
        );
    }


    /**
     * Экспорт Excel
     * */
    getUrlExport(): string {
        return `${this.config.httpUrl}/coordinate/excel`
    }


    /**
     * Экспорт txt
     * */
    getUrlExportTxt(): string {
        return `${this.config.httpUrl}/coordinate/txt`
    }


    /**
     * Обновить изображение
     * */
    setImageDetails(imagePosition: Position, imageWidthPx: number, canEdit: boolean) {
        const formData: FormData = new FormData();
        formData.append('imagePositionX', imagePosition.x.toString());
        formData.append('imagePositionY', imagePosition.y.toString());
        formData.append('imageWidthPx', imageWidthPx.toString());
        formData.append('canEdit', canEdit.toString());

        return this.http.post(`${this.config.httpUrl}/image/setImageDetails`, formData).pipe(
            catchError((error: HttpErrorResponse) => {
                return throwError(error);
            }),
            map(res => {
                return res;
            }),
        );
    }


    /**
     * Получить сессию
     * */
    getSession(): Observable<any> {
        return this.http.get(`${this.config.httpUrl}/session/get`).pipe(
            catchError((error: HttpErrorResponse) => {
                return throwError(error);
            }),
            map(res => {
                return res;
            }),
        );
    }
}
