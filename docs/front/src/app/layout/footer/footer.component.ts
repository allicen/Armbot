import {Component, OnInit} from '@angular/core';
import {HttpService} from "../../serviсes/http.service";
import {UntilDestroy, untilDestroyed} from "@ngneat/until-destroy";

@UntilDestroy()
@Component({
    selector: 'app-footer',
    templateUrl: './footer.component.html',
    styleUrls: ['./footer.component.less']
})
export class FooterComponent implements OnInit {
    title = 'docs';
    version: string = '';
    versionFrom: string = '';

    constructor(private httpService: HttpService) {
    }

    ngOnInit(): void {
        this.httpService.getAppVersion().pipe(untilDestroyed(this)).subscribe(data => {
            if (data?.details?.version) {
                this.version = data.details.version;
            }

            if (data?.details?.date) {
                this.versionFrom = data.details.date;
            }
        })
    }
}
