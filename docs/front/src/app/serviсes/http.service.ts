import {Injectable} from "@angular/core";
import {Observable, of, Subscription, throwError} from "rxjs";
import {HttpClient, HttpErrorResponse} from "@angular/common/http";
import {FileHandle} from "../pages/user-interface/dragDrop.directive";
import {Config} from "../config/config";
import {catchError, map} from "rxjs/operators";
import {Coordinate, ImagePosition} from "../model/models";


@Injectable({ providedIn: 'root' })
export class HttpService {

    constructor(private http: HttpClient, private config: Config){ }

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

    getImage(): Observable<any> {
      return this.http.get(`${this.config.httpUrl}/image/getImage`).pipe(

      );
    }

    removeImage(): Observable<any> {
      return this.http.get(`${this.config.httpUrl}/image/removeImage/`).pipe(
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


  saveSessionState(imageId: number = 0, imageSize: number = 0, imagePosition: ImagePosition): Observable<any> {
    return this.http.post(`${this.config.httpUrl}/session/save`,
      {imageId: imageId, imageSize: imageSize, imagePositionX: imagePosition.x, imagePositionY: imagePosition.y}).pipe(
      catchError((error: HttpErrorResponse) => {
        return throwError(error);
      }),
      map(res => {
        return res;
      }),
    );
  }

  getSession() {
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
