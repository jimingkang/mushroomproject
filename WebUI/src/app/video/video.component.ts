import { CommonModule } from '@angular/common';
import { Component } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { ProductService } from '../services/product.service';

@Component({
  selector: 'app-video',
  standalone: true,
  imports: [RouterOutlet,CommonModule],
  templateUrl: './video.component.html',
  styleUrl: './video.component.scss'
})
export class VideoComponent {
  isSidePanelVisible: boolean= false;
 
  categoryList: any [] = [];
  productsList: any [] = [];
  constructor(private productSrv: ProductService) {
    debugger;
  }
 
  
}

