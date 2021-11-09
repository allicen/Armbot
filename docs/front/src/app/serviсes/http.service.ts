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

    // uploadImage(file: FileHandle): Observable<any> {
    //
    //   const formData: FormData = new FormData();
    //   formData.append('file', file.file);
    //   formData.append('name', file.file.name);
    //   formData.append('contentType', file.file.type);
    //
    //   return this.http.post(`${this.config.httpUrl}/image/upload`, formData).pipe(
    //     map(res => {
    //       return res;
    //     })
    //   );
    // }

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

    getImage(): Observable<any> {
      return this.http.get(`${this.config.httpUrl}/image/getImage`).pipe(

      );
    }

    removeSession(): Observable<any> {
      return this.http.get(`${this.config.httpUrl}/session/remove/`).pipe(
        map(res => {
          return res;
        })
      );
    }

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

  removeAllCoordinates(): Observable<any> {
    return this.http.get(`${this.config.httpUrl}/coordinate/removeAll`).pipe(
      map(res => {
        return res;
      })
    );
  }

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

  getUrlExport(): string {
      return `${this.config.httpUrl}/coordinate/excel`
  }

  getUrlExportTxt(): string {
    return `${this.config.httpUrl}/coordinate/txt`
  }


  // uploadImage(file: FileHandle): Observable<any> {
  //
  //   const formData: FormData = new FormData();
  //   formData.append('file', file.file);
  //   formData.append('name', file.file.name);
  //   formData.append('contentType', file.file.type);
  // }


  // @Part Optional<byte[]> file,
  // @Part Optional<String> name,
  // @Part Optional<String> contentType,
  // @Part Optional<Integer> imagePositionX,
  // @Part Optional<Integer> imagePositionY,
  // @Part Optional<Integer> imageWidthPx,
  // @Part Optional<Integer> cursorArea)


  saveSessionState(file: FileHandle | null,
                   imagePositionX: number = 0,
                   imagePositionY: number = 0,
                   imageWidthPx: number = 0,
                   cursorArea: number = 0): Observable<any> {

    const formData: FormData = new FormData();
    formData.append('file', file?.file || '');
    formData.append('name', file?.file?.name || '');
    formData.append('contentType', file?.file?.type || '');
    formData.append('imagePositionX', imagePositionX.toString());
    formData.append('imagePositionY', imagePositionY.toString());
    formData.append('imageWidthPx', imageWidthPx.toString());
    formData.append('cursorArea', cursorArea.toString());

    return this.http.post(`${this.config.httpUrl}/session/save`, formData).pipe(
      catchError((error: HttpErrorResponse) => {
        return throwError(error);
      }),
      map(res => {
        return res;
      }),
    );
  }


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
