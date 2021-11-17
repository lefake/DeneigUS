import { Component, HostListener, ViewChild } from '@angular/core';
import { NgxJoystickComponent,JoystickEvent } from 'ngx-joystick';
import { RosServiceService } from './ros-service.service';


import {  JoystickOutputData } from 'nipplejs';
import { JoyMessage, NumberArrayMessage } from 'ngx-roslib';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.css']
})

export class AppComponent {
  title = 'angular-tour-of-heroes';
  @ViewChild('staticJoystic') staticJoystick: NgxJoystickComponent | undefined;



  @HostListener('document:keydown.space', ['$event'])
  handleKeyboardEvent(event: KeyboardEvent) {
    this.service.deadMan();
  }
 
  constructor(private service: RosServiceService) { }
  public test(){
     this.service.subscribeTopic();
     
  }
    position: number[] = [];
    output: JoyMessage = {
      header: { }
    };
  onMoveStatic(event: JoystickEvent) {

    this.position[0]=event.data.raw.position.x;
    this.position[1]=event.data.raw.position.y;
    this.output.axes=this.position;
 
    this.service.sendPos(this.output);
    console.log('la position est ' + this.position);
    console.log('le output est ' + this.output.axes);
  }

  public battery ="10%" //this.ControlesService.listenTopicDebugMot();

  mapOptions: google.maps.MapOptions = {
    center: { lat: 45.37847040811782, lng: -71.92573075811909 },
    zoom : 17,

 }
 

  flightPlanCoordinates = [
  { lat: 45.37783600271528, lng: -71.92486853547857},
  { lat:45.377745096754474, lng: -71.92460836121214 },
  { lat: 45.37750487822728, lng:-71.92490206309022},
  { lat: 45.37764853844953, lng: -71.92507975943198 },
  { lat: 45.37783600271528, lng: -71.92486853547857},
];


 polylineOptions: google.maps.PolylineOptions = {
  strokeOpacity : 1,
  strokeColor: "#FF0000",
  path: this.flightPlanCoordinates,

 }


 icon : google.maps.Icon = 
 {
   url:'https://maps.google.com/mapfiles/kml/shapes/parking_lot_maps.png'
 }


 marker = {
    position: { lat: 45.37847040811782, lng: -71.92573075811909 },
 
     
    options: {
      animation: google.maps.Animation.DROP,
      icon: this.icon
    }
 }


  
}
