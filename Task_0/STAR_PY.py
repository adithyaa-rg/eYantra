''''
# Team ID:          eYRC#HB#2008
# Theme:            Hologlyph Bots
# Author List:      Adithyaa RG
# Filename:         STAR_PY.py
# Functions:        star_print
# Global variables: test_cases, number_of_lines
'''

# Print rows
def star_print(n):
    """
    Purpose:
    ---
    This function prints the required pattern

    Input Arguments:
    ---
    n (int): The number of rows

    Returns:
    ---
    None

    Example call:
    ---
    star_print(15)
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