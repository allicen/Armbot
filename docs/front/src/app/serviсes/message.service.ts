import {Injectable} from "@angular/core";
import {BehaviorSubject, Observable} from "rxjs";

@Injectable({ providedIn: 'root' })
export class MessageService {

  private messageGeneral$: BehaviorSubject<string> = new BehaviorSubject<string>('');
  private messageGeneralIsError$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false);

  private messageBottom$: BehaviorSubject<string> = new BehaviorSubject<string>('');
  private messageBottomIsError$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false);

  private messageImport$: BehaviorSubject<string> = new BehaviorSubject<string>('');
  private messageImportIsError$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false);

  constructor() {
  }

  getMessageGeneral(): Observable<string> {
    return this.messageGeneral$.asObservable();
  }

  setMessageGeneral(message: string): void {
    this.messageGeneral$.next(message);
  }

  getMessageBottom(): Observable<string> {
    return this.messageBottom$.asObservable();
  }

  setMessageBottom(message: string): void {
    this.messageBottom$.next(message);
  }

  getMessageImport(): Observable<string> {
    return this.messageImport$.asObservable();
  }

  setMessageImport(message: string): void {
    this.messageImport$.next(message);
  }
}
