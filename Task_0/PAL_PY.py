'''
This script is code stub for CodeChef problem code PAL_PY
Filename:      PAL_PY.py
Created:       14/09/2023
Last Modified: 14/09/2023
Author:        e-Yantra Team - eYRC#HB#2008
'''

# Take the input for the palindromes
def palindrome_input(num_inputs):
    word_array = []
    for i in range(num_inputs):
        input_user = input()

        # Checking for the length constraint
        if len(input_user) < 2 or len(input_user)>100:
            break
        word_array += []
    return word_array

# Checking for the palindromes
def palindrome_check(word_array):
    for i in word_array:
        if i == i[-1::-1]:
            print('It is a palindrome')
        else:
            print('It is not a palindrome')

# Call the MAIN function
if __name__== '__main__':

    # Take in T - number of inputs
    num_inputs = int(input())

    # Checking the constraints for number of inputs
    if 1 <= num_inputs <= 100:

        # Get the word array and check for palindrome
        word_array = palindrome_input(num_inputs)
        palindrome_check(word_array)
