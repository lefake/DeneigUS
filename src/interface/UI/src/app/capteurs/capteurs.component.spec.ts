import { ComponentFixture, TestBed } from '@angular/core/testing';

import { CapteursComponent } from './capteurs.component';

describe('CapteursComponent', () => {
  let component: CapteursComponent;
  let fixture: ComponentFixture<CapteursComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ CapteursComponent ]
    })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(CapteursComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
