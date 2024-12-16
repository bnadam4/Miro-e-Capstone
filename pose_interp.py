#!/usr/bin/python3
#
# Author: Bryce Adam
# Date created: December 16, 2024
# Last modified: December 16, 2024
#
# Main default program for MiRo when it is not undergoing some pre-programmed activity

import numpy as np

def interp_pos(init_pos, end_pos, steps):
    positions = []
    positions.append(init_pos)

    eqns = np.array([
        [0, 0, 0, 0, 0, 1],
        [steps**5, steps**4, steps**3, steps**2, steps, 1],
        [0, 0, 0, 0, 1, 0],
        [5 * steps**4, 4 * steps**3, 3 * steps**2, 2 * steps, 1, 0],
        [0, 0, 0, 2, 0, 0],
        [20 * steps**3, 12 * steps**2, 6 * steps, 2, 0, 0]])
    
    eqns_inv = np.linalg.inv(eqns)
    smat = np.array([init_pos, end_pos, 0, 0, 0, 0])
    coeffs = np.dot(eqns_inv, smat)

    for pos in range(1, int(steps)):
        interp_pos = coeffs[0]*pos**5 + coeffs[1]*pos**4 + coeffs[2]*pos**3 + coeffs[3]*pos**2 + coeffs[4]*pos + coeffs[5]
        positions.append(interp_pos)

    positions.append(end_pos)

    return positions

