''''
# Team ID:          eYRC#HB#2008
# Theme:            Hologlyph Bots
# Author List:      Haricharan B
# Filename:         DIST1_PY.py
# Functions:        compute_distance
# Global variables: test_cases, x1, y1, x2, y2
'''
 
# Importing required modules
import sys, os, math

# Function to calculate Euclidean distance between two points
def compute_distance(x1, y1, x2, y2):
    """
    Purpose:
    ---
    This function calculates the Euclidean distance between two points

    Input Arguments:
    ---
    x1 (int): x-coordinate of first point
    y1 (int): y-coordinate of first point
    x2 (int): x-coordinate of second point
    y2 (int): y-coordinate of second point

    Returns:
    ---
    None

    Example call:
    ---
    compute_distance(5,6,7,8)
    """

    # Using formula for euclidean distance between two points in plane
    distance = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    # Using f-string to print up to two decimal places
    print(f"Distance: {distance:.2f}")

# Main function
if __name__ == '__main__':
    
    # Take the T (test_cases) input
    test_cases = int(input())

    for i in range(test_cases):

        # Taking input of x1, y1, x2, y2
        x1, y1, x2, y2 = map(int, sys.stdin.readline().split())    
        compute_distance(x1, y1, x2, y2)