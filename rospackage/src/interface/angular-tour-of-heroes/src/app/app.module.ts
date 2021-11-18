import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { NgxRoslibService } from 'ngx-roslib';
import { NgxJoystickModule } from 'ngx-joystick';
import {GoogleMapsModule} from '@angular/google-maps';
import { NgTerminalModule } from 'ng-terminal';
import { FlexLayoutModule } from '@angular/flex-layout';

@NgModule({
  declarations: [
    AppComponent,
  ],
  imports: [
    BrowserModule,
    FlexLayoutModule,
    AppRoutingModule,
    NgxJoystickModule,
    GoogleMapsModule,
    NgTerminalModule,
    

  ],
  providers: [NgxRoslibService],
  bootstrap: [AppComponent]
})
export class AppModule { }
