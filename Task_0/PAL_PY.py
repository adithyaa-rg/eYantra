'''
This script is code stub for CodeChef problem code PAL_PY
Filename:      PAL_PY.py
Created:       14/09/2023
Last Modified: 14/09/2023
Author:        e-Yantra Team - eYRC#HB#2008
'''
# Checking for the palindromes
def palindrome_check(string):
    """
    This function checks if the string is a palindrome or not

    Parameters:
    string (str): The string to be checked

    Returns:
    None

    Example:
    >>> palindrome_check('racecar')
    It is a palindrome
    """

    # Check if the string is a palindrome, by checking if the string is equal to its reverse    
    if string == string[::-1]:
        print('It is a palindrome')
    else:
        print('It is not a palindrome')

# Call the MAIN function
if __name__== '__main__':

    # Take in T - number of testcases
    test_cases = int(input())

    # Iterate over the testcases
    for _ in range(test_cases):  

        # Take in the string  
        string = input()

        # Call the function to check for palindrome
        palindrome_check(string)