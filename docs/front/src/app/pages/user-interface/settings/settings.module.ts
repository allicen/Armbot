import { NgModule } from '@angular/core';
import {SettingsComponent} from "./settings.component";
import {RouterModule} from "@angular/router";
import {MatListModule} from "@angular/material/list";
import {CommonModule} from "@angular/common";
import {MatFormFieldModule} from "@angular/material/form-field";
import {MatChipsModule} from "@angular/material/chips";
import {MatAutocompleteModule} from "@angular/material/autocomplete";
import {MatOptionModule} from "@angular/material/core";
import {MatIconModule} from "@angular/material/icon";
import {ReactiveFormsModule} from "@angular/forms";
import {DragDropModule} from "@angular/cdk/drag-drop";
import {MatButtonModule} from "@angular/material/button";
import {MatInputModule} from "@angular/material/input";
import {MatExpansionModule} from "@angular/material/expansion";

@NgModule({
  declarations: [
    SettingsComponent,
  ],
    imports: [
        RouterModule.forChild([{
            path: '',
            component: SettingsComponent
        }]),
        MatListModule,
        CommonModule,
        MatFormFieldModule,
        MatChipsModule,
        MatAutocompleteModule,
        MatOptionModule,
        MatIconModule,
        ReactiveFormsModule,
        DragDropModule,
        MatButtonModule,
        MatInputModule,
        MatExpansionModule,
    ],
  exports: [SettingsComponent],
  providers: [],
  bootstrap: [SettingsComponent]
})
export class SettingsModule { }
