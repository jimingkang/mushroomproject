import { CommonModule } from '@angular/common';
import { Component, Input, OnInit, signal } from '@angular/core';
import { MatCardModule } from '@angular/material/card';
import { MatToolbarModule } from '@angular/material/toolbar';
import { MatButtonModule } from '@angular/material/button';
import { VideoComponent } from '../..//video/video.component';
import { ProductService } from '../../services/product.service';
import { RpiService } from '../../services/rpi.service';
import { SlidepanelComponent } from '../../slidepanel/slidepanel.component';

import { Pipe, PipeTransform } from '@angular/core';
    import { DomSanitizer, SafeHtml, SafeStyle, SafeScript, SafeUrl, SafeResourceUrl } from '@angular/platform-browser';

import { SafePipe } from '../../safe.pipe';
    
type CardContent = {
  title: string;
  description: string;
  imageUrl: string;
};



@Component({
  selector: 'app-card-view',
  standalone: true,
  templateUrl: './card-view.component.html',
  styles: [
    `
      img {
        width: 100%;
        height: 200px;
        object-fit: cover;
      }

      .responsive-grid {
        display: grid;
        //grid-template-columns: repeat(auto-fill, minmax(250px, 1fr));
        gap: 24px;
      }
    `,
  ],
  imports: [SafePipe,SlidepanelComponent,CommonModule, MatCardModule, MatToolbarModule, MatButtonModule,VideoComponent],
})

export class CardViewComponent implements OnInit {
  @Input({ required: true })
  ip!: string;
  cards = signal<CardContent[]>([]);
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

  itsSafe: SafeUrl | undefined;

  // Private properties
 // private safePipe: SafePipe = new SafePipe(this.domSanitizer);
  public ngAfterViewInit() {
    console.log(this.ip);
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
    constructor(private productSrv: ProductService,private rpiSrc: RpiService,private domSanitizer: DomSanitizer) {
      
    }
    safeIp(){
      return this.domSanitizer.bypassSecurityTrustUrl(this.ip);
    }
    ngOnInit(): void {
    // this.getProducts();
    //  this.getALlCategory();
    //this.productsList.
    //this.itsSafe = this.safePipe.transform(this.ip, 'url');
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