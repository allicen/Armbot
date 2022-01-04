import {Injectable} from "@angular/core";
import {EMPTY, Observable, throwError} from "rxjs";
import {HttpClient, HttpErrorResponse} from "@angular/common/http";
import {FileHandle} from "../pages/user-interface/command-lib/import-image/dragDrop.directive";
import {Config} from "../config/config";
import {catchError, map} from "rxjs/operators";
import {Coordinate, Position} from "../model/models";
import {MessageService} from "./message.service";


@Injectable({ providedIn: 'root' })
export class HttpService {

    constructor(private http: HttpClient, private config: Config, private messageService: MessageService){ }

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
     * Импорт команд из файла
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
     * Удалить сеанс
     * */
    removeSession(): Observable<any> {
        return this.http.get(`${this.config.httpUrl}/session/remove/`).pipe(
            map(res => {
                return res;
            })
        );
    }


    /**
     * Обновить команду
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
     * Обновить команду
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
     * Удалить все команды
     * */
    removeAllCoordinates(): Observable<any> {
        return this.http.get(`${this.config.httpUrl}/coordinate/removeAll`).pipe(
            map(res => {
                return res;
            })
        );
    }


    /**
     * Удалить команду
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
        return `${this.config.httpUrl}/coordinate/excel`;
    }


    /**
     * Экспорт txt
     * */
    getUrlExportTxt(): string {
        return `${this.config.httpUrl}/coordinate/txt`;
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
     * Получить сеанс
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


    /**
     * Записать строку в файл запуска
     * */
    saveLaunchFileRow(coordinateId: number, delay: number): Observable<any> {
        const formData: FormData = new FormData();
        formData.append('coordinateId', coordinateId.toString());
        formData.append('delay', delay.toString());

        return this.http.post(`${this.config.httpUrl}/file/save`, formData).pipe(
            catchError((error: HttpErrorResponse) => {
                return throwError(error);
            }),
            map(res => {
                return res;
            }),
        );
    }


    /**
     * Удалить строку из файла запуска
     * */
    removeLaunchFileRow(fileRow: number): Observable<any> {
        return this.http.get(`${this.config.httpUrl}/file/remove/${fileRow}`).pipe(
            catchError((error: HttpErrorResponse) => {
                return throwError(error);
            }),
            map(res => {
                return res;
            }),
        );
    }


    /**
     * Обновить задержку для строки из файла запуска
     * */
    updateDelayFileRow(fileRowId: number, delay: number): Observable<any> {
        const formData: FormData = new FormData();
        formData.append('fileRowId', fileRowId.toString());
        formData.append('delay', delay.toString());

        return this.http.post(`${this.config.httpUrl}/file/setDelay`, formData).pipe(
            catchError((error: HttpErrorResponse) => {
                return throwError(error);
            }),
            map(res => {
                return res;
            }),
        );
    }


    /**
     * Обновить сортировку из файла запуска
     * */
    updateSortOrderFileRow(sortOrderRows: string): Observable<any> {
        return this.http.post(`${this.config.httpUrl}/file/sort`, sortOrderRows).pipe(
            catchError((error: HttpErrorResponse) => {
                return throwError(error);
            }),
            map(res => {
                return res;
            }),
        );
    }


    /**
     * Экспорт txt
     * */
    exportLaunchFileTxt(): string {
        return `${this.config.httpUrl}/file/txt`;
    }


    /**
     * Экспорт сеанса в json
     * */
    exportSessionJson(): string {
        return `${this.config.httpUrl}/session/export`;
    }


    /**
     * Импорт сеанса из json
     * */
    importSessionJson(file: File): Observable<any> {
        return this.http.post(`${this.config.httpUrl}/session/import`, file).pipe(
          catchError((error: HttpErrorResponse) => {
            this.messageService.setMessageImport('Ошибка импорта. Убедитесь, что выбради файл в формате .json');
            return throwError(error);
          }),
            map(res => {
                return res;
            })
        );
    }

    /**
     * Сохранить размер точки
     * */
    setCursorPoint(size: number): Observable<any> {
        return this.http.post(`${this.config.httpUrl}/session/setCursorSize`, {cursorSize: size}).pipe(
            catchError((error: HttpErrorResponse) => {
                return throwError(error);
            }),
            map(res => {
                return res;
            }),
        );
    }

    /**
     * Получить данные для запуска робота
     * */
    runRobot(): Observable<any> {
        return this.http.get(`${this.config.httpUrl}/robot/run`).pipe(
            map(res => {
                return res;
            })
        );
    }

    /**
     * Создать пустую сессию
     * */
    sessionCreate(): Observable<any> {
        return this.http.get(`${this.config.httpUrl}/session/create`).pipe(
            map(res => {
                return res;
            })
        );
    }

    /**
     * Скачать файл запуска
     * */
    getLaunchFile(): Observable<any> {
        return this.http.get(this.exportLaunchFileTxt()).pipe(
            map(res => {
                return res;
            })
        );
    }

  /**
   * Получить номер версии
   * */
    getAppVersion(): Observable<any> {
        return this.http.get(`${this.config.httpUrl}/version/get`).pipe(
            map(res => {
                return res;
            })
        );
    }

    /**
     * Получить конфиги робота
     * */
    getArmbotConfigs(): Observable<any> {
        return this.http.get(`${this.config.httpUrl}/robot/config`).pipe(
            map(res => {
                return res;
            })
        );
    }
}
