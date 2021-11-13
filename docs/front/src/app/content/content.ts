import {Injectable} from "@angular/core";

@Injectable({ providedIn: 'root' })
export class Content {
  jsonInfo: string = '{\n' +
    '  "sessionId": 1,\n' +
    '  "workOption": "uploadImage",\n' +
    '  "settings": null,\n' +
    '  "coordinateList": [\n' +
    '    {\n' +
    '      "id": 3,\n' +
    '      "name": "coordinate-2",\n' +
    '      "x": 155,\n' +
    '      "y": 8,\n' +
    '      "z": 0,\n' +
    '      "unit": "MM"\n' +
    '    }\n' +
    '  ],\n' +
    '  "launchFileRowList": [\n' +
    '    {\n' +
    '      "id": 1,\n' +
    '      "coordinateId": 1,\n' +
    '      "sortOrder": 0,\n' +
    '      "delay": 0\n' +
    '    }\n' +
    '  ],\n' +
    '  "image": {\n' +
    '    "name": "0-70.jpg",\n' +
    '    "contentType": "image/jpeg",\n' +
    '    "imageByte": [-1],\n' +
    '    "imageWidthPx": 493,\n' +
    '    "imagePositionY": 75,\n' +
    '    "imagePositionX": 194,\n' +
    '    "canEdit": false\n' +
    '  }\n' +
    '}'
}
