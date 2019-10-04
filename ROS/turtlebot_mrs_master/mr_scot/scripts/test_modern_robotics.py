#!/usr/bin/env python

import numpy as np
from modern_robotics import *
def verify_SE3( M ):
	is_SE3 = 1 #1 is SE3, 0 isn't SE3
	e = .0005
	for r in range(0,4):
		for c in range(0,4):
			if M[r,c] > e:
				is_SE3 = 0
	return is_SE3;

test1 = np.array([(1.5,2,3,1), (4,5,6,1),(4,5,6,1),(4,5,6,1)])

print(verify_SE3(test1))

test2 = np.array([(0,0,0,0), (0,0,0,0),(0,0,0,0),(0,0,0,0)])

print(verify_SE3(test2))


#position
#x: -0.262516419213
#y: 0.0956618903586
#z: 0.853000397219

#orientation:
#x: 0.964715453833
#y: 0.0345085876614
#z: 0.256778933071
#w: 0.0468810200799
