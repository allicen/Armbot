import {Component, OnInit} from '@angular/core';
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {HttpService} from "../../../serviсes/http.service";
import {Config} from "../../../config/config";
import {ArmbotService} from "../../../serviсes/armbot.service";
import {CameraImage} from "../../../model/models";

@UntilDestroy()
@Component({
  selector: 'app-settings',
  templateUrl: './settings.component.html',
  styleUrls: ['./settings.component.less']
})
export class SettingsComponent implements OnInit {

    constructor(private httpService: HttpService, private config: Config, private armbotService: ArmbotService) { }

    settings: any;
    panelOpenState: boolean = false;
    imageCamera: CameraImage = {
        data: null,
        encoding: null,
        width: 600,
        height: 300
    };

    ngOnInit(): void {
        this.httpService.getArmbotConfigs().pipe(untilDestroyed(this)).subscribe(data => this.loadConfig(data));
        this.armbotService.getImageFromCamera().pipe(untilDestroyed(this)).subscribe(data =>  {
          this.imageCamera = data;
          // console.log(data)
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
