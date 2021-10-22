import {Injectable} from "@angular/core";
import {Observable, Subscription} from "rxjs";
import {HttpClient} from "@angular/common/http";
import {FileHandle} from "../pages/user-interface/dragDrop.directive";


@Injectable({ providedIn: 'root' })
export class HttpService {
    httpUrl: string = 'http://0.0.0.0:9080';

    image: any = null
    message: string | undefined

    constructor(private http: HttpClient){ }

    uploadImage(file: FileHandle): Subscription {

      const formData: FormData = new FormData();
      formData.append('file', file.file);
      formData.append('name', file.file.name);
      formData.append('contentType', file.file.type);

      return this.http.post(`${this.httpUrl}/image/upload`, formData).subscribe(data => {

      });
    }
}
