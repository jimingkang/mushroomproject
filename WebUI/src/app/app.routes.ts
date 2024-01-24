import { Routes } from '@angular/router';
import { AppComponent } from './app.component';
import { LoginComponent } from './login/login.component';

import { WoodframeComponent } from './woodframe/woodframe.component';
import { HeaderComponent } from './header/header.component';


export const routes: Routes = [
   
    {
        path:'',
        redirectTo:'warehouse',
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
                path:'warehouse',
                component:WoodframeComponent
            },
          
        ]
    },
];
