'''
This script is code stub for CodeChef problem code IFFOR1_PY
Filename:      IFFOR1_PY.py
Created:       14/09/2023
Last Modified: 14/09/2023
Author:        e-Yantra Team - eYRC#HB#2008
'''

import sys

# Take the T (test_cases) input
T = int(sys.stdin.readline())

# Check the constraints on T
if 1 <= T <= 25:
    list = []
    for i in range(T):
        # Take the n input
        input_n = int(sys.stdin.readline())

        # Check the constraints on n
        if 0 <= input_n <= 100:
            l+=[input_n]
        else:
            break

    # Making the lists
    for i in l:
        temp_list = []
        for j in range(i):

            # Check the conditions and add to list
            if j == 0:
                temp_list += [j+3]
            elif j%2 == 0:
                temp_list += [2*j]
            else:
                temp_list += [j**2]
        print(*temp_list)
