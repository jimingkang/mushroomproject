import { ComponentFixture, TestBed } from '@angular/core/testing';

import { MushroomComponent } from './mushroom.component';

describe('MushroomComponent', () => {
  let component: MushroomComponent;
  let fixture: ComponentFixture<MushroomComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [MushroomComponent]
    })
    .compileComponents();
    
    fixture = TestBed.createComponent(MushroomComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
