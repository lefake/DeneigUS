import { Component, OnInit } from '@angular/core';
import { ControlesService } from './../services/controles.service';



@Component({
  selector: 'app-capteurs',
  templateUrl: './capteurs.component.html',
  styleUrls: ['./capteurs.component.css']
})


export class CapteursComponent implements OnInit {

  

  constructor(private ControlesService: ControlesService) { }

  ngOnInit(): void {
  }
  public battery = this.ControlesService.listenTopicDebugMot();

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
