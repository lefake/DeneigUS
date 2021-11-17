import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { NgxRoslibService } from 'ngx-roslib';
import { NgxJoystickModule } from 'ngx-joystick';
import {GoogleMapsModule} from '@angular/google-maps';

@NgModule({
  declarations: [
    AppComponent,
  ],
  imports: [
    BrowserModule,
    AppRoutingModule,
    NgxJoystickModule,
    GoogleMapsModule

  ],
  providers: [NgxRoslibService],
  bootstrap: [AppComponent]
})
export class AppModule { }
