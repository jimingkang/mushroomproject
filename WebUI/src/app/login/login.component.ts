import { Component } from '@angular/core';

import { CommonModule } from '@angular/common';
import { Router, RouterLink, RouterOutlet } from '@angular/router';
import { FormsModule } from '@angular/forms';

@Component({
  selector: 'app-login',
  standalone: true,
  imports: [FormsModule,RouterOutlet,CommonModule],
  templateUrl: './login.component.html',
  styleUrl: './login.component.scss',
  
})
export class LoginComponent {
  loginObj: any = {
    userName: '',
    password: ''
  };
  constructor(private router: Router){}

  onLogin() {
    console.log(this.loginObj.userName == "admin")
    console.log(this.loginObj.password)
    //if(this.loginObj.userName == "admin" && this.loginObj.password == "334455") {
 
   //   this.router.navigateByUrl('/landing')

   // } else {
   //   alert('Wrong Credentials')
   // }
  }

}
