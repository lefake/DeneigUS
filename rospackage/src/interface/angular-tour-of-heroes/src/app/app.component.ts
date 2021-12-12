import { Component, HostListener, ViewChild } from '@angular/core';
import { NgxJoystickComponent,JoystickEvent } from 'ngx-joystick';
import { RosServiceService } from './ros-service.service';


import { NumberArrayMessage, NumberMessage } from 'ngx-roslib';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.css']
})

export class AppComponent {
  title = 'angular-tour-of-heroes';
  deadManActivated = false;
  @ViewChild('staticJoystic') staticJoystick: NgxJoystickComponent | undefined;

  isTrue :NumberMessage = {
    data: 1
  }

  position: number[] = [];
  output: NumberArrayMessage = {
    data:[]
  };

  angle: NumberArrayMessage = {
    data: [2.0],
  }


  @HostListener('window:keydown.space', ['$event'])
  handleKeyboardEvent(event: KeyboardEvent) {
    event.preventDefault();
    this.isTrue.data=1;
    this.service.deadMan(this.isTrue);
    this.deadManActivated=true;
  }
 
  constructor(private service: RosServiceService) { }
  public test(){
     this.service.subscribeTopic();
     
  }

  onMoveStatic(event: JoystickEvent) {

    

    this.position[1]=event.data.vector.x;
    this.position[0]=event.data.vector.y;
    this.output.data=this.position;
    

    if(this.deadManActivated){
      this.service.sendPos(this.output);
      this.deadManActivated=false;
    }
  }

  onEndStatic(event:JoystickEvent){

    console.log("release")
    this.output.data = [0,0];
    this.service.sendPos(this.output);
  
    this.isTrue.data=0;
    this.service.deadMan(this.isTrue);
  }

  onMoveStaticChute(event: JoystickEvent) {

    this.angle.data[0]=event.data.angle.degree;
    this.service.sendAngle(this.angle);
 
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
