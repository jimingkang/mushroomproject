import { Component } from '@angular/core';

import { CommonModule } from '@angular/common';
import { Router, RouterLink, RouterOutlet } from '@angular/router';
import { FormsModule } from '@angular/forms';
import { AngularFireAuth, AngularFireAuthModule } from '@angular/fire/compat/auth';
import firebase from 'firebase/compat/app';
import { AuthService } from '../services/auth.service';

import { GoogleAuthProvider, GithubAuthProvider, FacebookAuthProvider} from '@angular/fire/auth'
const firebaseConfig = {
  apiKey: "AIzaSyCYPDBw1Ub8v33C2BB2PhRUIg3a57opPz4",
  authDomain: "mushroom-30b6a.firebaseapp.com",
  projectId: "mushroom-30b6a",
  storageBucket: "mushroom-30b6a.appspot.com",
  messagingSenderId: "44586902461",
  appId: "1:44586902461:web:8652274cec4148d03568d4"
};
@Component({
  selector: 'app-login',
  standalone: true,
  imports: [FormsModule,RouterOutlet,CommonModule,AngularFireAuthModule,

    AngularFireAuthModule, //Firebase imports
    AngularFireAuthModule,],
  templateUrl: './login.component.html',
  styleUrl: './login.component.scss',
  
})
export class LoginComponent {
  loginObj: any = {
    userName: '',
    password: ''
  };
  constructor(private fireauth:AngularFireAuth, private router: Router){}

  onLogin() {
    //console.log(this.loginObj.userName == "admin")
    //console.log(this.loginObj.password=="334455")
    this.fireauth.signInWithEmailAndPassword(this.loginObj.userName,this.loginObj.password).then( res => {
      localStorage.setItem('token','true');
      console.log(res)

    

      if(res.user?.emailVerified == false) {
        this.router.navigate(['/warehouse']);
      } else {
        alert("error for login");
        //this.router.navigate(['']);
      }

  }, err => {
      alert(err.message);
      this.router.navigate(['/login']);
  })
//if(this.loginObj.userName == "admin" && this.loginObj.password == "334455") {
 
// this.router.navigateByUrl('/warehouse')
//} else {
 //    alert('Wrong Credentials')
 // }
  }
  onGoogleLogin(){
  //this.auth.login(this.loginObj.userName,this.loginObj.password);
this.fireauth.signInWithPopup(new firebase.auth.GoogleAuthProvider());

   // this.auth.googleSignIn();
  }

}
