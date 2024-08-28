import { CommonModule } from '@angular/common';
import { Component } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { ProductService } from '../services/product.service';
import * as ROSLIB from 'roslib';
import { DomSanitizer, SafeResourceUrl } from '@angular/platform-browser';
import { MushroomComponent } from '../mushroom/mushroom.component';
@Component({
  selector: 'app-video',
  standalone: true,
  imports: [RouterOutlet,CommonModule,MushroomComponent],
  templateUrl: './video.component.html',
  styleUrl: './video.component.scss'
})
export class VideoComponent {
  isSidePanelVisible: boolean= false;

dataUrl: SafeResourceUrl = "";
 
  categoryList: any [] = [];
  productsList: any [] = [];
  ros: ROSLIB.Ros | undefined;
  listener: ROSLIB.Topic<ROSLIB.Message> | undefined;
  pose: HTMLImageElement | undefined;
  public width = 156;//495;
  public height = 240;//445;
 
 
   private cx!: CanvasRenderingContext2D;
  imagePath: any;
  constructor(private productSrv: ProductService,private _sanitizer: DomSanitizer) {

    this.ros = new ROSLIB.Ros({
      url : 'ws://172.27.34.65:9090'
      });
      this.ros.on('connection', function() {
      console.log('Connected to websocket server.');
      });
      this.ros.on('error', function(error) {
      console.log('Error connecting to websocket server: ', error);
      });
      this.ros.on('close', function() {
      console.log('Connection to websocket server closed.');
      });
      // ----------------------
       this.listener = new ROSLIB.Topic({
      ros : this.ros,
      name : '/camera/camera/color/image_rect_raw',
      messageType : 'sensor_msgs/msg/Image'
      });
      
      this.listener.subscribe((message:any) => {
        this.pose = new Image();
      console.log(message.data);
      let image_sub = <HTMLImageElement> document.querySelector('#image_sub')
      var data=document.getElementById('image_sub')
      //image_sub.src=this._sanitizer.bypassSecurityTrustResourceUrl("data:image/jpg;base64, "+message.data);
      //this.pose.src=message.data
      //image_sub.src="'data:image/jpg;base64,'"+message.data;
      this.dataUrl=this._sanitizer.bypassSecurityTrustResourceUrl("data:image/bgr8;base64, "+message.data);
   
      //image_sub.src=this.dataUrl
      //sleep(100)
      //this.cx.drawImage(this.pose, 0, 0, this.width, this.height);

      });
  }
 
  
}


