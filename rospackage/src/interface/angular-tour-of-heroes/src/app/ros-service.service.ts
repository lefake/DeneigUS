import { Injectable } from '@angular/core';
import { JoyMessage, NgxRoslibService, NumberMessage, Rosbridge } from 'ngx-roslib';
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

  
    public sendPos(joyData:any){
    let request = joyData;
     

      const joy = new RosTopic<JoyMessage>({
        ros: this.rbServer,
        name: '/prop2',
        messageType:'sensor_msgs/Joy',
        
    });
    joy.advertise();
 
    joy.subscribe(() => {
     
     
    });
  
      
      joy.publish(request);
      joy.unsubscribe();
    
  
    }

    public deadMan(){
      const isTrue :NumberMessage = {
        data: 1
      }
       
  
        const joy = new RosTopic<NumberMessage>({
          ros: this.rbServer,
          name: '/deadman',
          messageType:'std_msgs/int32',
          
      });
  
    
      joy.advertise();
   
      joy.subscribe(() => {
          
       
      });
      joy.publish(isTrue);
      joy.unsubscribe();


   
    
      }
}
