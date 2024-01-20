import { Routes } from '@angular/router';
import { AppComponent } from './app.component';
import { LoginComponent } from './login/login.component';
import { LandingComponent } from './landing/landing.component';
import { WoodframeComponent } from './woodframe/woodframe.component';
import { HeaderComponent } from './header/header.component';


export const routes: Routes = [
   
    {
        path:'',
        redirectTo:'landing',
        pathMatch:'full'
    },
    {
        path:'login',
        component:LoginComponent
    },
    
    {
        path:'',
        component:HeaderComponent,
        children: [
            {
                path:'landing',
                component:WoodframeComponent
            },
          
        ]
    },
];
