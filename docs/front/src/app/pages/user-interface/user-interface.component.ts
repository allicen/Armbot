import {Component, ElementRef, OnInit, ViewChild} from '@angular/core';
import {StorageService} from "../../serviсes/storage.service";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {Router} from "@angular/router";
import {SessionService} from "../../serviсes/session.service";
import {OpenDialogComponent} from "./open-dialog/open-dialog.component";
import {MatDialog} from "@angular/material/dialog";
import {HttpService} from "../../serviсes/http.service";

@UntilDestroy()
@Component({
  selector: 'app-user-interface',
  templateUrl: './user-interface.component.html',
  styleUrls: ['./user-interface.component.less']
})
export class UserInterfaceComponent implements OnInit {

  tab: string = 'user-interface';
  sessionExists: boolean = false;
  exportSessionUrl: string = '';

  @ViewChild("exportJson") exportJson: ElementRef | undefined;

  constructor(private storageService: StorageService,
              private router: Router,
              private sessionService: SessionService,
              private dialog: MatDialog,
              private httpService: HttpService) {
  }

  ngOnInit(): void {
    this.storageService.getUserInterfaceActiveTab().pipe(untilDestroyed(this)).subscribe(data => {
      if (data !== '') {
        this.tab = data;
      }
    });

      this.tab = this.storageService.getUserInterfaceTab(this.router.url);

      this.sessionService.getSession().pipe(untilDestroyed(this)).subscribe();
      this.sessionService.getSessionExists().pipe(untilDestroyed(this)).subscribe(data => this.sessionExists = data);
      this.exportSessionUrl = this.httpService.exportSessionJson();
  }

  changeTab(tab: string) {
    this.storageService.setUserInterfaceActiveTab(tab);
  }

  removeSession() {
    this.dialog.open(OpenDialogComponent, {
      data: {title: 'Завершить сеанс?',
        text: 'Будут удалены все координаты и загруженное изображение. Действие отменить нельзя.',
        type: 'remove_session'}
    });
  }

  exportSession() {
    this.exportJson?.nativeElement?.click();
  }
}
