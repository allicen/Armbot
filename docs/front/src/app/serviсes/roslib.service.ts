import {UntilDestroy} from "@ngneat/until-destroy";
import {Injectable} from "@angular/core";
import * as ROSLIB from 'roslib';

@UntilDestroy()
@Injectable({ providedIn: 'root' })
export class RosLibService {

  ros: any

  constructor() {
    this.ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });
  }

  connect() {
    console.log('test')
  }


}
