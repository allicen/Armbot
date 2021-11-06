import {Injectable} from "@angular/core";
import {BehaviorSubject, Observable} from "rxjs";
import {HttpService} from "./http.service";
import {DomSanitizer} from "@angular/platform-browser";
import {FileHandle} from "../pages/user-interface/command-lib/import-image/dragDrop.directive";
import {Position} from "../model/models";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";

@UntilDestroy()
@Injectable({ providedIn: 'root' })
export class ImageService {
  constructor(private httpService: HttpService, private sanitizer: DomSanitizer) {
  }

  private image$: BehaviorSubject<any> = new BehaviorSubject<any>(null);
  private imagePosition$: BehaviorSubject<Position> = new BehaviorSubject<Position>({x: 0, y: 0});
  private imageEditAllowed$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(true);
  private imageUploadRequired$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(true);
  private imageWidth$: BehaviorSubject<number> = new BehaviorSubject<number>(0);

  getImage(): Observable<any> {
    this.getImageFromServer();
    return this.image$.asObservable();
  }

  setImage(image: FileHandle): void {
    this.httpService.uploadImage(image).pipe(untilDestroyed(this)).subscribe(data => {
      if (data.status === 'OK') {
        this.getImageFromServer();
      }
    });
  }

  deleteImage(): void {
    this.image$.next(null);
  }

  getImageFromServer() {
    this.httpService.getImage().pipe(untilDestroyed(this)).subscribe((data: any) => {
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

  getImageUploadRequired(): Observable<boolean> {
    return this.imageUploadRequired$.asObservable();
  }

  setImageUploadRequired(required: boolean): void {
    this.imageUploadRequired$.next(required);
  }

  getImageWidth(): Observable<number> {
    return this.imageWidth$.asObservable();
  }

  setImageWidth(width: number): void {
    this.imageWidth$.next(width);
  }


}
