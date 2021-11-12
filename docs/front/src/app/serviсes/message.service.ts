import {Injectable} from "@angular/core";
import {BehaviorSubject, Observable} from "rxjs";

@Injectable({ providedIn: 'root' })
export class MessageService {

    private messageImport$: BehaviorSubject<string> = new BehaviorSubject<string>('');
    private importErrors$: BehaviorSubject<string[]> = new BehaviorSubject<string[]>([]);
    private coordinateMessage$: BehaviorSubject<string> = new BehaviorSubject<string>('');
    private coordinateMessageIsError$: BehaviorSubject<boolean> = new BehaviorSubject<boolean>(false);

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

    setCoordinateMessage(message: string): void {
        this.coordinateMessage$.next(message);
    }

    setCoordinateMessageIsError(error: boolean): void {
        this.coordinateMessageIsError$.next(error);
    }

    clearVariable(): void {
        this.messageImport$.next('');
        this.importErrors$.next([]);
        this.coordinateMessage$.next('');
        this.coordinateMessageIsError$.next(false);
        this.importErrors = [];
        this.importMessage = '';
    }
}
