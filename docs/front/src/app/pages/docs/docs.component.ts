import {Component, ViewEncapsulation} from '@angular/core';

@Component({
  selector: 'app-docs',
  templateUrl: './docs.component.html',
  styleUrls: ['./docs.component.less',  '../../layout/main/main.component.less'],
  encapsulation: ViewEncapsulation.None
})
export class DocsComponent {

  constructor() {

  }

  userInterfaceOn: boolean = false;

  selectors:  {[id: string]: string} = {
    'errors': 'Решение проблем',
    'help': 'Справка',
  };

  objectKeys(selectors: {}) {
    return Object.keys(selectors);
  }
}
