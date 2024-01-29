import { Component } from '@angular/core';

import { initializeApp } from "firebase/app";
import { AngularFireModule } from "@angular/fire/compat";

const firebaseConfig = {
  apiKey: "AIzaSyCYPDBw1Ub8v33C2BB2PhRUIg3a57opPz4",
  authDomain: "mushroom-30b6a.firebaseapp.com",
  projectId: "mushroom-30b6a",
  storageBucket: "mushroom-30b6a.appspot.com",
  messagingSenderId: "44586902461",
  appId: "1:44586902461:web:8652274cec4148d03568d4"
};

// Initialize Firebase
const app = AngularFireModule.initializeApp(firebaseConfig);

@Component({
  selector: 'app-firebase-login',
  standalone: true,
  imports:   [],
  templateUrl: './firebase-login.component.html',
  styleUrl: './firebase-login.component.scss'
})
export class FirebaseloginComponent {

}
