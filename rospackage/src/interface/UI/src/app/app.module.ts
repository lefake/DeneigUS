import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { CapteursComponent } from './capteurs/capteurs.component';
import { StatusComponent } from './status/status.component';
import { DebugComponent } from './debug/debug.component';
import { ControllesComponent } from './controlles/controlles.component';
import { GoogleMapsModule } from '@angular/google-maps'
import { DemoMaterialModule } from './material-module';
import { NgTerminalModule } from 'ng-terminal';
import { NgxJoystickModule } from 'ngx-joystick';
import { NgxRoslibService } from 'ngx-roslib';



@NgModule({
  declarations: [
    AppComponent,
    CapteursComponent,
    StatusComponent,
    ControllesComponent,
    DebugComponent
  ],
  imports: [
    BrowserModule,
    AppRoutingModule,
    BrowserAnimationsModule,
    DemoMaterialModule,
    GoogleMapsModule,
    NgTerminalModule,
    NgxJoystickModule,
    
    
  ],
  exports: [
    DemoMaterialModule
  ],
  providers: [NgxRoslibService],
  bootstrap: [AppComponent]
})
export class AppModule { }
