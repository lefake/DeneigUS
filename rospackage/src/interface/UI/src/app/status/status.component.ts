

import {ThemePalette} from '@angular/material/core';
import { Component, Input,NgModule, Directive, } from '@angular/core';
import { Subject } from 'rxjs';



@Component({
  selector: 'app-status',
  templateUrl: './status.component.html',
  styleUrls: ['./status.component.css'],
})


export class StatusComponent{
  color: ThemePalette = 'warn';

  
  public data  = new Subject<string>();
  obj =   {
    data:{
     test: "ceci est un test"
    }
  }


  public clickMe(){
    console.log("je suis dans la methode");
    this.data.next(JSON.stringify(this.obj));

  }

}
