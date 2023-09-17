''''
# Team ID:          eYRC#HB#2008
# Theme:            Hologlyph Bots
# Author List:      Haricharan B
# Filename:         APLAM1_PY.py
# Functions:        generate_AP
# Global variables: test_cases, a1, d, n, AP_series, sqr_AP_series, sum_sqr_AP_series
'''

# Import reduce module
from functools import reduce

# Function to generate the A.P. series
def generate_AP(a1, d, n):
    """
    Purpose:
    ---
    This function generates the A.P. series

    Input Arguments:
    ---
    a1 (int): First term of the A.P. series
    d (int): Common difference of the A.P. series
    n (int): Number of terms in the A.P. series

    Returns:
    ---
    AP_series (list): A.P. series

    Example call:
    generate_AP(1, 2, 3)
    """

    AP_series = [a1]

    for _ in range(1, n):
        AP_series.append(AP_series[-1] + d)    

    return AP_series


# Main function
if __name__ == '__main__':
    
    # take the T (test_cases) input
    test_cases = int(input())

    for _ in range(test_cases):    

        # Taking input of a1, d, n
        a1, d, n = map(int, input().split())

        # Generating the AP series
        AP_series = generate_AP(a1, d, n)
        print(*AP_series, sep = " ")

        # Generating squares of elements using map function
        sqr_AP_series = list(map (lambda x: x**2, AP_series))
        print(*sqr_AP_series, sep = " ")

        # Generating sum of squares using reduce function
        sum_sqr_AP_series = reduce (lambda a, b: a + b, sqr_AP_series)
        print(sum_sqr_AP_series)

