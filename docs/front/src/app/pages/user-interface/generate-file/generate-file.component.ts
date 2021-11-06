import {Component, ElementRef, OnInit, ViewChild} from '@angular/core';
import {Coordinate} from "../../../model/models";
import {StorageService} from "../../../serviсes/storage.service";
import {HttpService} from "../../../serviсes/http.service";
import {MatAutocompleteSelectedEvent} from "@angular/material/autocomplete";
import {MatChipInputEvent} from "@angular/material/chips";
import {COMMA, ENTER} from "@angular/cdk/keycodes";
import {FormControl} from "@angular/forms";
import {Observable} from "rxjs";
import {map, startWith} from "rxjs/operators";
import {CdkDragDrop, moveItemInArray} from "@angular/cdk/drag-drop";

@Component({
  selector: 'app-generate-file',
  templateUrl: './generate-file.component.html',
  styleUrls: ['./generate-file.component.less']
})
export class GenerateFileComponent implements OnInit {

  coordinateList: Coordinate[] = [];

  selectable = true;
  removable = true;
  separatorKeysCodes: number[] = [ENTER, COMMA];
  commandCtrl = new FormControl();
  filteredCommand: Observable<Coordinate[]> | undefined;
  commands: Coordinate[] = [];

  @ViewChild('coordinateInput') coordinateInput: ElementRef<HTMLInputElement> | undefined;


  constructor(private storageService: StorageService, private httpService: HttpService) {

    this.filteredCommand = this.commandCtrl.valueChanges.pipe(
      startWith(null),
      map((item: string | null) => (item ? this._filter(item) : this.coordinateList.slice())),
    );
  }

  ngOnInit(): void {
      this.httpService.getSession().subscribe((data: any) => {
        if (data.status === 'SUCCESS' && data.details) {
          if (data.details.coordinateList) {
            this.storageService.setCoordinateList(data.details.coordinateList);
          }
        }
      });

      this.storageService.getCoordinateList().subscribe(data => {
        this.coordinateList = data;
      });

  }

  add(event: MatChipInputEvent): void {

    const value = (event.value || '').trim();
    const cIndex = this.coordinateList.findIndex(c => c.name === value);

    if (cIndex > 0) {
      this.commands.push(this.coordinateList[cIndex]);
    }
    event.chipInput!.clear();

    this.commandCtrl.setValue(null);
  }

  choice(id: number): void {
    const cIndex = this.coordinateList.findIndex(c => c.id == id);
    this.commands.push(this.coordinateList[cIndex]);
  }

  remove(index: number): void {
      const cIndex = this.commands.findIndex(c => c.id === index);
      this.commands.splice(cIndex, 1);
  }

  selected(event: MatAutocompleteSelectedEvent): void {
    const name = event.option.viewValue;
    const coordinate = this.coordinateList.filter(item => item.name === name);

    if (coordinate.length > 0) {
      this.commands.push(coordinate[0]);
    }

    if (!this.coordinateInput) {
      return;
    }

    this.coordinateInput.nativeElement.value = '';
    this.commandCtrl.setValue(null);
  }

  private _filter(value: string): Coordinate[] {
    const filterValue = value.toLowerCase();
    return this.coordinateList.filter(item => item.name.toLowerCase().includes(filterValue));
  }



  drop(event: CdkDragDrop<Coordinate[]>) {
    moveItemInArray(this.commands, event.previousIndex, event.currentIndex);
  }

}
