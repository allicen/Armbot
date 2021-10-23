import {Injectable} from "@angular/core";
import {Observable, Subscription} from "rxjs";
import {HttpClient} from "@angular/common/http";
import {FileHandle} from "../pages/user-interface/dragDrop.directive";
import {Config} from "../config/config";
import {map} from "rxjs/operators";


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
}
