import {Injectable} from "@angular/core";
import {BehaviorSubject, Observable} from "rxjs";
import {HttpService} from "./http.service";
import {DomSanitizer} from "@angular/platform-browser";
import {FileHandle} from "../pages/user-interface/command-lib/import-image/dragDrop.directive";
import {Position} from "../model/models";
import {repeatGroups} from "@angular/compiler/src/shadow_css";

@Injectable({ providedIn: 'root' })
export class ImageService {
  constructor(private httpService: HttpService, private sanitizer: DomSanitizer) {
  }

  private image$: BehaviorSubject<any> = new BehaviorSubject<any>(null);
  private imagePosition$: BehaviorSubject<Position> = new BehaviorSubject<Position>({x: 0, y: 0});
  private imageEditAllowed$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(true);

  getImage(): Observable<any> {
    this.getImageFromServer();
    return this.image$.asObservable();
  }

  setImage(image: FileHandle): void {
    this.httpService.uploadImage(image).pipe().subscribe(data => {
      if (data.status === 'OK') {
        this.getImageFromServer();
      }
    });
  }

  getImageFromServer() {
    this.httpService.getImage().subscribe((data: any) => {
      if (data.status === 'OK') {
        this.image$.next(this.sanitizer.bypassSecurityTrustResourceUrl(`data:${data.image.contentType};base64,${data.image.imageByte}`));
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
    return this.imageEditAllowed$.asObservable();
  }

  setImageEditAllowed(allowed: boolean): void {
    this.imageEditAllowed$.next(allowed);
  }


}
