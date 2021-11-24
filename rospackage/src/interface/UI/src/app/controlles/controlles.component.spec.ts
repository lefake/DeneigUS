import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ControllesComponent } from './controlles.component';

describe('ControllesComponent', () => {
  let component: ControllesComponent;
  let fixture: ComponentFixture<ControllesComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ControllesComponent ]
    })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(ControllesComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
