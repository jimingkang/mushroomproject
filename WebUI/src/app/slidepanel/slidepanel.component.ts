

import { Component, EventEmitter, Input, Output } from '@angular/core';
import {
  trigger,
  state,
  style,
  animate,
  transition,
} from '@angular/animations';
import { VideoComponent } from '../video/video.component';
import { RpiService } from '../services/rpi.service';

@Component({
  selector: 'app-slidepanel',
  standalone: true,
  imports: [VideoComponent],
  templateUrl: './slidepanel.component.html',
  styleUrl: './slidepanel.component.scss',
  animations: [
    trigger('fadeSlideInRight', [
      transition(':enter', [
        style({ opacity: 0, transform: 'translateX(100%)' }),
        animate('300ms', style({ opacity: 1, transform: 'translateX(0)' })),
      ]),
      transition(':leave', [
        animate('300ms', style({ opacity: 0, transform: 'translateX(100%)' })),
      ]),
    ]),
  ],
})
export class SlidepanelComponent {
  @Input() isOpen = true;
  @Input() headerText = 'Slide Panel Header';
  @Output() onClose = new EventEmitter();
  constructor(private rpiservice:RpiService){}

  onClosePanel() {
    this.onClose.emit(false);
  }
  xforward(){
    this.rpiservice.xfarward().subscribe((res:any)=>{
      console.log(res)
    });

  }
  xbackward(){
    console.log("x backward")
    this.rpiservice.xbackward().subscribe((res:any)=>{
     
      console.log(res)
    });

  }
  ybackward(){
    console.log("y backward")
    this.rpiservice.ybackward().subscribe((res:any)=>{
      console.log(res)
    });

  }
  yforward(){
    console.log(" y forward")
    this.rpiservice.yfarward().subscribe((res:any)=>{
      console.log(res)
    });

  }
  home(){
    console.log("home")
    const res=this.rpiservice.home().subscribe((res:any)=>{
      console.log(res)
    });
    

    
  }

  scan(){
    console.log("scan")
    this.rpiservice.scan().subscribe((res:any)=>{
      console.log(res)
    });

  }
}