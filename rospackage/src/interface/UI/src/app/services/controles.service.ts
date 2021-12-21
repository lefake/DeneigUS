
import { Injectable, Component } from '@angular/core';

import { NgxRoslibService, Rosbridge, RosoutMessage, RosTopic } from 'ngx-roslib';



@Injectable({
  providedIn: 'root'
})
export class ControlesService {

  rbServe: Rosbridge = new Rosbridge;
  


  constructor(public roslibService: NgxRoslibService) {
    this.rbServe = roslibService.connect('http://localhost:9090');  // Enter your Rosbridge URL here

   }

   public listenTopicDebugMot(){


  const rosout = new RosTopic<RosoutMessage>({
    ros: this.rbServe,
    name: '/debug_mot',
    messageType: 'std_msgs/Float32MultiArray',
});
rosout.subscribe((msg) => {
    console.log('Received a /rosout message:', msg);
});
   }
  
}


