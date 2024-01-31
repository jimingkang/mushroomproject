import { HttpClient } from '@angular/common/http';
import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class RpiService {

  constructor(private http:HttpClient) { }

  farward(){
    this.http.get(`RpiHttp.API_END_POINT+RpiHttp.METHODS.XFarward`);
  }
}


export const RpiHttp = {
  API_END_POINT:'http://192.168.0.100:5000/',
  METHODS: {
      XFarward: 'xforward',
      Backward: 'backfarward',
 
  }
}