import {Injectable} from "@angular/core";
import {BehaviorSubject, Observable} from "rxjs";
import {HttpService} from "./http.service";
import {DomSanitizer} from "@angular/platform-browser";
import {FileHandle} from "../pages/user-interface/command-lib/import-image/dragDrop.directive";
import {Position} from "../model/models";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";

@UntilDestroy()
@Injectable({ providedIn: 'root' })
export class SessionService {
  constructor(private httpService: HttpService, private sanitizer: DomSanitizer) {
  }

  private sessionExists$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false);
  private image$: BehaviorSubject<any> = new BehaviorSubject<any>(null);
  private imagePosition$: BehaviorSubject<Position> = new BehaviorSubject<Position>({x: 0, y: 0});
  private imageEditAllowed$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(true);
  private imageUploadRequired$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(true);
  private imageWidth$: BehaviorSubject<number> = new BehaviorSubject<number>(0);
  private canEditImage$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(true);


  getSession(): Observable<any> {
    this.uploadSession();
    return this.sessionExists$.asObservable();
  }

  getImage(): Observable<any> {
    return this.image$.asObservable();
  }

  setSession(image: FileHandle): void {
    this.httpService.saveSessionState(image, 0, 0, 0, 0)
      .pipe(untilDestroyed(this)).subscribe(data => {
      this.fillFieldsSession(data);
    });
  }

  uploadSession(): void {
    this.httpService.getSession()
      .pipe(untilDestroyed(this))
      .subscribe(data => {
        this.fillFieldsSession(data);
    });
  }

  fillFieldsSession(data: any) {
    if (data.status === 'SUCCESS') {
      this.sessionExists$.next(true);
    }

    if (data.status === 'SUCCESS' && data.details.image) {
      this.image$.next(this.sanitizer.bypassSecurityTrustResourceUrl(`data:${data.details.image.contentType};base64,${data.details.image.imageByte}`));
      this.imagePosition$.next({x: data.details.image.imagePositionX, y: data.details.image.imagePositionY});
      this.imageWidth$.next(data.details.image.imageWidthPx);
      this.canEditImage$.next(data.details.image.canEdit);
    }
  }

  deleteImage(): void {
    // this.image$.next(null);
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
