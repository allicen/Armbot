import {Component, ElementRef, OnInit, ViewChild} from '@angular/core';
import {Coordinate, LaunchFileRow} from "../../../model/models";
import {CdkDragDrop, moveItemInArray} from "@angular/cdk/drag-drop";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {SessionService} from "../../../serviсes/session.service";
import {HttpService} from "../../../serviсes/http.service";
import {StorageService} from "../../../serviсes/storage.service";

@UntilDestroy()
@Component({
  selector: 'app-generate-file',
  templateUrl: './generate-file.component.html',
  styleUrls: ['./generate-file.component.less']
})
export class GenerateFileComponent implements OnInit {

    coordinateList: Coordinate[] = [];
    commands: LaunchFileRow[] = [];
    rowHasError: boolean = false;
    messageRow: string = '';
    rowId: number = -1;
    prevDelay: number = 0;
    maxId: number = 0;
    exportTxtUrl: string = '';
    coordinateDelete: number = -1;

    @ViewChild('coordinateInput') coordinateInput: ElementRef<HTMLInputElement> | undefined;
    @ViewChild("commandList") commandList: ElementRef | undefined;
    @ViewChild("exportTxt") exportTxt: ElementRef | undefined;

    constructor(private sessionService: SessionService,
                private httpService: HttpService,
                private storageService: StorageService) { }

    ngOnInit(): void {
        this.exportTxtUrl = this.httpService.exportLaunchFileTxt();
        this.sessionService.getCoordinateList().pipe(untilDestroyed(this)).subscribe(data => this.coordinateList = data);
        this.sessionService.getLaunchFileRow().pipe(untilDestroyed(this)).subscribe(data => {
            this.commands = data;
        });
        this.sessionService.getNextFileRowId().pipe(untilDestroyed(this)).subscribe(data => this.maxId = data);
        this.storageService.getCoordinateDelete().pipe(untilDestroyed(this)).subscribe(data => {
            if (data != -1) {
                const cIndex = this.commands.findIndex(c => c.id === data);
                this.commands.splice(cIndex, 1);
            }
            this.storageService.setCoordinateDelete(-1);
        });
    }

    choice(id: number): void {
        const cIndex = this.coordinateList.findIndex(c => c.id == id);
        const next: number = this.maxId + 1;
        const command: LaunchFileRow = {id: next, coordinate: this.coordinateList[cIndex], delay: 0, sortOrder: next};
        this.sessionService.addLaunchFileRowList(command);
        this.clearMessage();
    }

    remove(index: number): void {
        this.httpService.removeLaunchFileRow(index).pipe().subscribe(data => {
            if (data.status === 'SUCCESS') {
                const cIndex = this.commands.findIndex(c => c.id === index);
                this.commands.splice(cIndex, 1);
                this.clearMessage();
            }
        });
    }

    drop(event: CdkDragDrop<Coordinate[]>) {
        const previousIndex: number = event.previousIndex;
        const currentIndex: number = event.currentIndex;

        moveItemInArray(this.commands, previousIndex, currentIndex);
        this.sessionService.launchFileRowListSort(previousIndex, currentIndex, event.previousIndex, event.currentIndex);
        this.clearMessage();
    }

    changeDelay(item: LaunchFileRow, delay: string) {
      this.rowId = item.id;

      if (!this.isNumber(delay)) {
          this.messageRow = 'Не число!';
          this.rowHasError = true;

          if (this.commandList) {
              this.commandList.nativeElement.value;
              this.commandList.nativeElement.querySelector(`#delay-${item.id}`).value = this.prevDelay;
          }
          return;
      }
        this.rowHasError = false;
        this.messageRow = 'Сохранено!';

        setTimeout(() => {
            this.rowId = -1;
            this.messageRow = '';
        }, 3000);

        this.prevDelay = Number(delay);
        this.sessionService.changeDelay(item.id, delay);
    }

    savePrevDelay(delay: string) {
        this.prevDelay = this.isNumber(delay) ? Number(delay) : 0;
    }

    isNumber(value: any) {
        return value.match(/^[\d\\.]+$/);
    }

    clearMessage() {
        this.messageRow = '';
        this.rowId = -1;
    }

    exportFileTxt() {
        this.exportTxt?.nativeElement.click();
    }
}
