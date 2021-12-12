import { Injectable } from '@angular/core';
import {  NgxRoslibService, NumberArrayMessage, NumberMessage, Rosbridge } from 'ngx-roslib';
import { RosTopic, RosoutMessage } from 'ngx-roslib';





@Injectable({
  providedIn: 'root'
})
export class RosServiceService {

  rbServer: Rosbridge;

    constructor(public roslibService: NgxRoslibService) {
     
        this.rbServer = this.roslibService.connect('http://localhost:9090/');  // Enter your Rosbridge URL here
    }

    public subscribeTopic(){

      const rosout = new RosTopic<RosoutMessage>({
        ros: this.rbServer,
        name: '/rosout',
    messageType: 'rosgraph_msgs/Log',
    });
    rosout.subscribe((msg: any) => {
        console.log('Received a /rosout message:', msg);
    });
    }

  
    public sendPos(joyData:NumberArrayMessage){
   
      const joy = new RosTopic<NumberArrayMessage>({
        ros: this.rbServer,
        name: '/prop',
        messageType:'std_msgs/float32multiarray',
        
    });
    joy.advertise();
 
    joy.subscribe(() => {
     
     
    });
  
      
      joy.publish(joyData);

      console.log("la position publiée est: " + joyData.data)
      joy.unsubscribe();
    
  
    }

    public deadMan(data:NumberMessage){
     
       
  
        const joy = new RosTopic<NumberMessage>({
          ros: this.rbServer,
          name: '/deadman',
          messageType:'std_msgs/int32',
          
      });
  
    
      joy.advertise();
   
      joy.subscribe(() => {
          
       
      });
      joy.publish(data);
      console.log("deadman publie : " + data.data)
      joy.unsubscribe();   
    
      }

      
    public sendAngle(angle:any){
     
  
      const joy = new RosTopic<NumberArrayMessage>({
        ros: this.rbServer,
        name: '/chute',
        messageType:'std_msgs/float32multiarray',
        
    });

  
    joy.advertise();
 
    joy.subscribe(() => {
        
     
    });
    joy.publish(angle);
    joy.unsubscribe();   
  
  }

}
