'''
This script is code stub for CodeChef problem code IFFOR1_PY
Filename:      IFFOR1_PY.py
Created:       14/09/2023
Last Modified: 14/09/2023
Author:        e-Yantra Team - eYRC#HB#2008
'''

import sys

def check_input(n):
    """
    This function checks the input and prints the required output

    Parameters:
    n (int): The number of elements in the list

    Returns:
    None

    Example:
    >>> check_input(5)
    3 1 4 9 8
    """

    # Initializing a list
    temp_list = []

    # Iterating over the range of n
    for i in range(n):
        # Checking the conditions and add to list
        if i == 0:
            temp_list.append(i+3)
        elif i%2 == 0:
            temp_list.append(2*i)
        else:
            temp_list.append(i**2)
    print(*temp_list)

# Calling the main function
if __name__== '__main__':

    # Taking in the number of testcases
    test_cases = int(input())

    # Iterating over the testcases
    for _ in range(test_cases):  

        input_number = int(sys.stdin.readline())
        check_input(input_number)
