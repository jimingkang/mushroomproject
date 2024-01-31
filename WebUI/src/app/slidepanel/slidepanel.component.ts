

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
    this.rpiservice.farward();

  }
}