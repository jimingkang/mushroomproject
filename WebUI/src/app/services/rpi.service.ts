import { HttpClient, HttpHeaders } from '@angular/common/http';
import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class RpiService {

  constructor(private http:HttpClient) { }

  xfarward(){
   return this.http.get(RpiHttp.API_END_POINT+RpiHttp.METHODS.XFarward);
  }
 xbackward(){
  return this.http.get(RpiHttp.API_END_POINT+RpiHttp.METHODS.XBackward);
  }
  yfarward(){
    return this.http.get(RpiHttp.API_END_POINT+RpiHttp.METHODS.YFarward);
  }
 ybackward(){
  return this.http.get(RpiHttp.API_END_POINT+RpiHttp.METHODS.YBackward);
  }
  movePosion(posion:any){
    const httpOptions = {
      headers: new HttpHeaders({ 'Content-Type': 'application/json' }),
    };
console.log(posion)
    return this.http.post(RpiHttp.REEMAN_END_POINT+RpiHttp.METHODS.MOVEPOSITION,{ "point": posion}, httpOptions);
   }

  home(){
   return this.http.get(RpiHttp.API_END_POINT+RpiHttp.METHODS.Home);
  }
 scan(){
  return this.http.get(RpiHttp.API_END_POINT+RpiHttp.METHODS.SCAN);
  }
  start(){
   return this.http.get(RpiHttp.API_END_POINT+RpiHttp.METHODS.START);
   
    }
    stop(){
     return this.http.get(RpiHttp.API_END_POINT+RpiHttp.METHODS.STOP);
     
      }
}


export const RpiHttp = {
  API_END_POINT:'http://172.27.34.65:5002',
  REEMAN_END_POINT:'http://172.27.34.70',
  METHODS: {
      START: '/start_scan',
      STOP: '/stop_scan',
      XFarward: '/move/custom?x=10',
      XBackward: '/move/custom?x=-10',
      YFarward: '/move/custom?y=10',
      YBackward: '/move/custom?y=-10',
      Home: '/move/home',
      SCAN: '/scan',
      MOVEPOSITION:'/cmd/nav_point'
 
  }
}