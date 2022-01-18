import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import { CapteursComponent } from './capteurs/capteurs.component';
import { ControllesComponent } from './controlles/controlles.component';
import { StatusComponent } from './status/status.component';
import { DebugComponent } from './debug/debug.component';

const routes: Routes = [{ path: 'capteurs', component: CapteursComponent },
{ path: 'controlles', component: ControllesComponent },
{ path: 'status', component: StatusComponent },
{ path: 'debug', component: DebugComponent },
];

@NgModule({
  imports: [RouterModule.forRoot(routes),],
  exports: [RouterModule]
})
export class AppRoutingModule {

  
 }
