''''
# Team ID:          eYRC#HB#2008
# Theme:            Hologlyph Bots
# Author List:      Adithyaa RG
# Filename:         PAL_PY.py
# Functions:        palindrome_check
# Global variables: test_cases, string
'''
# Checking for the palindromes
def palindrome_check(string):
    """
    Purpose:
    ---
    This function checks if the string is a palindrome or not

    Input Arguments:
    ---
    string (str): The string to be checked

    Returns:
    ---
    None

    Example call:
    ---
    palindrome_check('racecar')
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