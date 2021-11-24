import { TestBed } from '@angular/core/testing';

import { RosServiceService } from './ros-service.service';

describe('RosServiceService', () => {
  let service: RosServiceService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(RosServiceService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
