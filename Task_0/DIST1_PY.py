'''
This script is code stub for CodeChef problem code DIST1_PY
Filename:      DIST1_PY_solution.py
Created:       27/09/2021
Last Modified: 27/09/2021
Author:        e-Yantra Team
'''
 
# Importing required modules
import sys, os, math

# Function to calculate Euclidean distance between two points
def compute_distance(x1, y1, x2, y2):
    """
    This function calculates the Euclidean distance between two points

    Parameters:
    x1 (int): x-coordinate of first point
    y1 (int): y-coordinate of first point
    x2 (int): x-coordinate of second point
    y2 (int): y-coordinate of second point

    Returns:
    None

    """

    # Using formula for euclidean distance between two points in plane
    distance = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    # Using f-string to print up to two decimal places
    print(f"{distance:.2f}")

# Main function
if __name__ == '__main__':
    
    # Take the T (test_cases) input
    test_cases = int(input())

    for i in range(test_cases):

        # Taking input of x1, y1, x2, y2
        x1, y1, x2, y2 = map(int, sys.stdin.readline().split())    
        compute_distance(x1, y1, x2, y2)