import {Component, OnInit} from '@angular/core';
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {HttpService} from "../../../serviсes/http.service";
import {Config} from "../../../config/config";

@UntilDestroy()
@Component({
  selector: 'app-settings',
  templateUrl: './settings.component.html',
  styleUrls: ['./settings.component.less']
})
export class SettingsComponent implements OnInit {

    constructor(private httpService: HttpService, private config: Config) { }

    settings: any;

    ngOnInit(): void {
        this.httpService.getArmbotConfigs().pipe(untilDestroyed(this)).subscribe(data => {
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
        });
    }
}
