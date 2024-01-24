


import { CommonModule } from '@angular/common';
import { Component, OnInit } from '@angular/core';
import {  RouterOutlet } from '@angular/router';
import { ProductService } from '../services/product.service';
import { FormsModule } from '@angular/forms';

import "@babylonjs/core/Physics/physicsEngineComponent";
import { VideoComponent } from '../video/video.component';
import { SlidepanelComponent } from '../slidepanel/slidepanel.component';



@Component({
  selector: 'app-woodframe',
  standalone: true,
  imports: [SlidepanelComponent,CommonModule,FormsModule,VideoComponent,RouterOutlet],
  templateUrl: './woodframe.component.html',
  styleUrl: './woodframe.component.scss'
})

export class WoodframeComponent implements OnInit {
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
  productsList: any [] = [];

  constructor(private productSrv: ProductService) {
    
  }
  ngOnInit(): void {
   this.getProducts();
    this.getALlCategory();

  }
 
  getProducts() {
    this.productSrv.getProducts().subscribe((res:any)=>{
      this.productsList = res.data;
    })
  }

  getALlCategory() {
    this.productSrv.getCategory().subscribe((res:any)=>{
      this.categoryList = res.data;
    })
  }
  onUpdate() {
    this.productSrv.saveProduct(this.productObj).subscribe((res:any)=>{
      debugger;
      if(res.result) {
        alert("Product Created");
        this.getProducts();
      } else {
        alert(res.message)
      }
    })
  }
  onSave() {
    this.productSrv.saveProduct(this.productObj).subscribe((res:any)=>{
      debugger;
      if(res.result) {
        alert("Product Updated");
        this.getProducts();
      } else {
        alert(res.message)
      }
    })
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


  onEdit(item: any) {
    this.productObj = item;
    this.openSidePanel();
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
