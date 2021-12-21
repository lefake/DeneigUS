import { ControlesService } from './../services/controles.service';
import { Component, OnInit } from '@angular/core';


interface Mode {
  value: string;
  viewValue: string;
}

@Component({
  selector: 'app-controlles',
  templateUrl: './controlles.component.html',
  styleUrls: ['./controlles.component.css'],
})
export class ControllesComponent implements OnInit {
  modes: Mode[] = [
    {value: 'mode1', viewValue: 'mode 1'},
    {value: 'mode2', viewValue: 'mode 2'},
    {value: 'mode3', viewValue: 'mode 3'}
  ];
  constructor(private ControlesService: ControlesService) { }

  ngOnInit(): void {
  }

}
