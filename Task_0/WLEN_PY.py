'''
This script is code stub for CodeChef problem code DIST1_PY
Filename:      DIST1_PY_solution.py
Created:       27/09/2021
Last Modified: 27/09/2021
Author:        e-Yantra Team
'''
 
# Importing required modules
import sys, os, math

# Function to count number of words
def count_words(string):

    words = string[1:].split()
    lengths = map(len, words)

    print(*lengths, sep = ",")


# Main function
if __name__ == '__main__':
    
    # Take the T (test_cases) input
    test_cases = int(input())

    for i in range(test_cases):

        # Taking input of x1, y1, x2, y2
        string = sys.stdin.readline()
        count_words(string)