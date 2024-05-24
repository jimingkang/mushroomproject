import { Routes } from '@angular/router';
import { AppComponent } from './app.component';
import { LoginComponent } from './login/login.component';

import { WoodframeComponent } from './woodframe/woodframe.component';
import { HeaderComponent } from './header/header.component';
import {  FirebaseloginComponent } from './firebase-login/firebase-login.component';


export const routes: Routes = [
   
    {
        path:'',
        redirectTo:'login', //login
        pathMatch:'full'
    },

    {
        path:'login',
        component:LoginComponent
    },
    {
        path:'firebaselogin',
        component:FirebaseloginComponent
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
