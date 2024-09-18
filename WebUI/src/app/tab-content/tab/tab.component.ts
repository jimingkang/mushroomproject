
import { NgClass, NgFor, NgIf } from '@angular/common';
import { Component, OnInit, Input ,Output, EventEmitter } from '@angular/core';
@Component({
  selector: 'app-tabs',
  standalone: true,
  imports: [NgClass,NgIf,NgFor],
  templateUrl: './tab.component.html',
  styleUrl: './tab.component.scss'
})
export class TabComponent implements OnInit {
  @Input() tabsArray: string[] = [];
  @Output() onTabChange = new EventEmitter<Event>();
  activatedTab: any = 0;

  constructor() { }

  ngOnInit(): void {
  }
  setTab(index:number) {
    this.activatedTab = index;

    this.onTabChange.emit(this.activatedTab);
  }

}
