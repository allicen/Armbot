import {Component, OnInit} from '@angular/core';
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";
import {HttpService} from "../../../serviсes/http.service";
import {Config} from "../../../config/config";
import {ArmbotService} from "../../../serviсes/armbot.service";
import {CameraImage, RobotInfoFromCamera} from "../../../model/models";
import { DomSanitizer } from '@angular/platform-browser';
import {StorageService} from "../../../serviсes/storage.service";
import {RosArmbotService} from "../../../serviсes/roslib.service";

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
                private _sanitizer: DomSanitizer,
                private storage: StorageService,
                private roslibService: RosArmbotService) { }

    settings: any;
    panelOpenState: boolean = false;
    imageCamera: CameraImage = {
        data: null,
        encoding: null,
        width: 600,
        height: 300
    };
    imagePath: any;
    cameraImageShow: boolean = true;
    robotDiagnosticInfo: RobotInfoFromCamera | undefined;
    diagnosticMessage: string | undefined;
    returnDefaultPosition: boolean = false;

    ngOnInit(): void {
        this.httpService.getArmbotConfigs().pipe(untilDestroyed(this)).subscribe(data => this.loadConfig(data));
        this.armbotService.getImageFromCamera().pipe(untilDestroyed(this)).subscribe(data =>  {
            this.imageCamera = data;
            if (this.imageCamera.data) {
                this.imagePath = this._sanitizer.bypassSecurityTrustResourceUrl('data:image/jpg;base64,' + this.imageCamera.data);
            }
        });
        this.storage.getCameraImageShow().pipe(untilDestroyed(this)).subscribe(data => this.cameraImageShow = data);
        this.roslibService.getRobotDiagnosticInfo().pipe(untilDestroyed(this)).subscribe(data => {
            this.robotDiagnosticInfo = data;
            if (this.robotDiagnosticInfo.resultExists) {
                this.diagnosticMessage = "Информация получена";
            }
        });
        this.roslibService.getRobotReturnDefaultPosition().pipe(untilDestroyed(this)).subscribe(data => {
            this.returnDefaultPosition = data;
            if (!this.robotDiagnosticInfo?.resultExists && this.returnDefaultPosition) {
                this.diagnosticMessage = "Робот вернулся в исходное положение";
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

    diagnostics() {
        this.roslibService.robotDiagnostics();
    }

    round(val: number | null | undefined): number {
        if (!val) {
            return 0;
        }
        return Math.round(val * 100) / 100;
    }
}
