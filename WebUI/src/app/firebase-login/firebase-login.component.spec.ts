import { ComponentFixture, TestBed } from '@angular/core/testing';

import { FirebaseLoginComponent } from './firebase-login.component';

describe('FirebaseLoginComponent', () => {
  let component: FirebaseLoginComponent;
  let fixture: ComponentFixture<FirebaseLoginComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [FirebaseLoginComponent]
    })
    .compileComponents();
    
    fixture = TestBed.createComponent(FirebaseLoginComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
