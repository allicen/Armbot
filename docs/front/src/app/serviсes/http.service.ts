import {Injectable} from "@angular/core";
import {Observable, of, Subscription, throwError} from "rxjs";
import {HttpClient, HttpErrorResponse} from "@angular/common/http";
import {FileHandle} from "../pages/user-interface/dragDrop.directive";
import {Config} from "../config/config";
import {catchError, map} from "rxjs/operators";
import {Coordinate} from "../model/models";


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

  saveCoordinateToFile(coordinateList: Coordinate[]): Observable<any> {
    return this.http.post(`${this.config.httpUrl}/coordinate/save`, coordinateList).pipe(
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

}
