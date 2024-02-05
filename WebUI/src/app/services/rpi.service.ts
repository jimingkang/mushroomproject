import { HttpClient } from '@angular/common/http';
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

  home(){
   return this.http.get(RpiHttp.API_END_POINT+RpiHttp.METHODS.Home);
  }
 scan(){
  return this.http.get(RpiHttp.API_END_POINT+RpiHttp.METHODS.Scan);
  }
  
}


export const RpiHttp = {
  API_END_POINT:'http://172.26.52.46:5001',
  METHODS: {
      XFarward: '/move/custom?x=10',
      XBackward: '/move/custom?x=-10',
      YFarward: '/move/custom?y=10',
      YBackward: '/move/custom?y=-10',
      Home: '/move/home',
      Scan: '/scan',
 
  }
}