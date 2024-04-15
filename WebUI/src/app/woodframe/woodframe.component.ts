


import { CommonModule } from '@angular/common';
import { Component, Input, OnInit, signal } from '@angular/core';
import {  RouterOutlet } from '@angular/router';
import { ProductService } from '../services/product.service';
import { FormsModule } from '@angular/forms';

import "@babylonjs/core/Physics/physicsEngineComponent";
import { VideoComponent } from '../video/video.component';
import { SlidepanelComponent } from '../slidepanel/slidepanel.component';
import { RpiService } from '../services/rpi.service';

import { MatToolbarModule } from '@angular/material/toolbar';
import { MatTabsModule } from '@angular/material/tabs';
import { MatSlideToggleModule } from '@angular/material/slide-toggle';
import { CardViewComponent } from '../tab-content/card-view/card-view.component';
import { SidebarModule } from 'primeng/sidebar';
import { ThemePalette } from '@angular/material/core';
import { DxTabPanelModule } from 'devextreme-angular';
@Component({
  selector: 'app-woodframe',
  standalone: true,
  imports: [DxTabPanelModule,SidebarModule,SlidepanelComponent,MatSlideToggleModule,CommonModule,FormsModule,VideoComponent,RouterOutlet, MatToolbarModule,MatTabsModule, CardViewComponent,],
  templateUrl: './woodframe.component.html',
  styleUrl: './woodframe.component.scss'
})

export class WoodframeComponent implements OnInit {
  
  
onValueChanged($event: Event) {
throw new Error('Method not implemented.');
}

  selectedTabIndex = signal(0);
  prefetchTabs = signal(false);
  isSlidePanelOpen = false;
  isSidePanelVisible: boolean= false;
  productObj: IProduct = {
    "productId": 0,
    "productSku": "",
    "productName": "",
    "productPrice": 0,
    "productShortName": "",
    "productDescription": "",
    "createdDate": new Date(),
    "deliveryTimeSpan": "",
    "categoryId": 0,
    "productImageUrl": ""
  };
  categoryList: any [] = [];
  productsList: any [] = [this.productObj];
  scan=0;

  public width = 156;//495;
 public height = 240;//445;


  private cx!: CanvasRenderingContext2D;
  canvas: any;
activeLink: any;
links: any;
background: ThemePalette;
tabNames: any;

public ngAfterViewInit() {
  const canvas = document.querySelector('#my-canvas')
if (!(canvas instanceof HTMLCanvasElement)) return;
    const canvasEl: HTMLCanvasElement = canvas;
    this.cx = canvasEl.getContext('2d')!;
    let image = new Image();
    let pose = new Image();

    canvasEl.width = this.width;
    canvasEl.height = this.height;

    this.cx.lineWidth = 3;
    this.cx.lineCap = 'round';
    this.cx.strokeStyle = '#000';
     
  pose.src = "../../assets/pose.svg";
  image.src = "../../assets/map.png";
    image.onload = ()=> {
        this.cx.drawImage(image, 0, 0, this.width, this.height);
        this.cx.drawImage(pose,1.244/0.05,this.height-(50+0.107/0.05),10,10);
    }


}
  constructor(private productSrv: ProductService,private rpiSrc: RpiService) {
    
  }
  ngOnInit(): void {
  // this.getProducts();
  //  this.getALlCategory();
  //this.productsList.
  
  }
 
  getProducts() {
    this.productSrv.getProducts().subscribe((res:any)=>{
      this.productsList = res.data;
    })
  }

  getALlCategory() {
    //this.productSrv.getCategory().subscribe((res:any)=>{
   //   this.categoryList = res.data;
   // })
  }
  onStopScan() {
    this.rpiSrc.stop().subscribe((res:any)=>{
      debugger;
      if(res) {
        this.scan=0;
       
      } else {
        alert(res.message)
      }
    });

  }
  onStartScan() {
   this.rpiSrc.start().subscribe((res:any)=>{
    debugger;
    if(res) {
      console.log(res.result)
      this.scan=1;

   
    } else {
      alert(res.message)
    }
  });
  
  }
  onDelete(item: any) {
    const isDelete = confirm('Are you Sure want to delte');
    if(isDelete) {
      this.productSrv.deleteProduct(item.productId).subscribe((res:any)=>{
        debugger;
        if(res.result) {
          alert("Product Deleted");
          this.getProducts();
        } else {
          alert(res.message)
        }
      })
    }
  }


  onEdit(posion: any) {
  //  this.productObj = item;



  this.rpiSrc.movePosion(posion).subscribe((res:any)=>{

    if(res) {
      console.log(res)
    } else {
      alert(res.message)
    }
  });
    this.openSidePanel();
 
  }
  startScan(){
    this.rpiSrc.start();
  }


  openSidePanel() {
    this.isSidePanelVisible = true;
  }

  closeSidePanel() {
    this.isSidePanelVisible = false;
  }



}
export interface IProduct{
  productId: number,
  productSku: string,
  productName: string,
  productPrice: number,
  productShortName: string,
  productDescription: string,
  createdDate: Date,
  deliveryTimeSpan: string,
  categoryId: number,
  "productImageUrl": ""
}
