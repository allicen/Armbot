import {Injectable} from "@angular/core";
import {BehaviorSubject, Observable} from "rxjs";

@Injectable({ providedIn: 'root' })
export class MessageService {

  private messageImport$: BehaviorSubject<string> = new BehaviorSubject<string>('');
  private importErrors$: BehaviorSubject<string[]> = new BehaviorSubject<string[]>([]);

  importErrors = [];
  importMessage: string = '';

  constructor() {
  }

  getMessageImport(): Observable<string> {
    return this.messageImport$.asObservable();
  }

  setMessageImport(message: string): void {
    this.messageImport$.next(message);
  }

  getMessageImportErrors(): Observable<string[]> {
    return this.importErrors$.asObservable();
  }

  setMessageImportErrors(list: string[]): void {
    this.importErrors$.next(list);
  }
}
