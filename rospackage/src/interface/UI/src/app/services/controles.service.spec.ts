import { TestBed } from '@angular/core/testing';

import { ControlesService } from './controles.service';

describe('ControlesService', () => {
  let service: ControlesService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(ControlesService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
