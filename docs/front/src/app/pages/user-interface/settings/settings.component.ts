import {Component, OnInit} from '@angular/core';
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {HttpService} from "../../../serviсes/http.service";
import {Config} from "../../../config/config";
import {ArmbotService} from "../../../serviсes/armbot.service";
import {CameraImage} from "../../../model/models";
import { DomSanitizer } from '@angular/platform-browser';

@UntilDestroy()
@Component({
  selector: 'app-settings',
  templateUrl: './settings.component.html',
  styleUrls: ['./settings.component.less']
})
export class SettingsComponent implements OnInit {

    constructor(private httpService: HttpService,
                private config: Config,
                private armbotService: ArmbotService,
                private _sanitizer: DomSanitizer) { }

    settings: any;
    panelOpenState: boolean = false;
    imageCamera: CameraImage = {
        data: null,
        encoding: null,
        width: 600,
        height: 300
    };
    imagePath: any;

    ngOnInit(): void {
        this.httpService.getArmbotConfigs().pipe(untilDestroyed(this)).subscribe(data => this.loadConfig(data));
        this.armbotService.getImageFromCamera().pipe(untilDestroyed(this)).subscribe(data =>  {
            this.imageCamera = data;
            if (this.imageCamera.data) {
                this.imagePath = this._sanitizer.bypassSecurityTrustResourceUrl('data:image/jpg;base64,' + this.imageCamera.data);
            }
        });
    }

    configUpdate() {
        this.httpService.updateArmbotConfigs().pipe(untilDestroyed(this)).subscribe(data => this.loadConfig(data));
    }

    loadConfig(data: any): void {
        if (data?.details) {
            this.config.robotConfig.forEach(item => {
                let key = item.key;
                let configItem = data?.details.filter((x: { key: string; }) => x.key === key);
                if (configItem.length === 1) {
                    item.value = configItem[0].value;
                }
            });
        }
        this.settings = this.config.robotConfig;
    }

    returnDefaultPositionCamera() {
        this.armbotService.returnDefaultPositionCamera();
    }
}
