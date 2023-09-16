'''
This script is code stub for CodeChef problem code PAL_PY
Filename:      STAR_PY.py
Created:       14/09/2023
Last Modified: 14/09/2023
Author:        e-Yantra Team - eYRC#HB#2008
'''

# Print rows
def star_print(n):
    """
    This function prints the required pattern

    Parameters:
    n (int): The number of rows

    Returns:
    None

    """
    
    # Iterate over the range of n
    for j in range(n, 0, -1):
        for k in range(1, j + 1):
            if k%5 == 0:
                print("#",end = '')
            else:
                print("*",end = '')
        print()

# Call the MAIN function
if __name__== '__main__':

    # Taking in number of testcases
    test_cases = int(input())

    for _ in range(test_cases):

        # Take in the number
        number_of_lines = int(input())
        star_print(number_of_lines)