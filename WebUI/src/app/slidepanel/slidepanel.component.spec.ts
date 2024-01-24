import { ComponentFixture, TestBed } from '@angular/core/testing';

import { SlidepanelComponent } from './slidepanel.component';

describe('SlidepanelComponent', () => {
  let component: SlidepanelComponent;
  let fixture: ComponentFixture<SlidepanelComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [SlidepanelComponent]
    })
    .compileComponents();
    
    fixture = TestBed.createComponent(SlidepanelComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
