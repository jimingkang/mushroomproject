import { ComponentFixture, TestBed } from '@angular/core/testing';

import { WoodframeComponent } from './woodframe.component';

describe('WoodframeComponent', () => {
  let component: WoodframeComponent;
  let fixture: ComponentFixture<WoodframeComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [WoodframeComponent]
    })
    .compileComponents();
    
    fixture = TestBed.createComponent(WoodframeComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
