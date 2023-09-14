'''
This script is code stub for CodeChef problem code PAL_PY
Filename:      STAR_PY.py
Created:       14/09/2023
Last Modified: 14/09/2023
Author:        e-Yantra Team - eYRC#HB#2008
'''

# Take the input for the no of rows
def star_input(num_inputs):
    len_array = []
    for i in range(num_inputs):
        input_user = int(input())

        # Checking for the length constraint
        if input_user < 1 or input_user>100:
            break
        len_array += [input_user]
    return len_array

# Print rows
def star_print(len_array):
    for i in len_array:
        for j in range(-i,0,-1):
            for k in range(j):
                if k%5 == 0:
                    print("#",end = '')
                else:
                    print("*",end = '')
            print()

# Call the MAIN function
if __name__== '__main__':

    # Take in T - number of inputs
    num_inputs = int(input())

    # Checking the constraints for number of inputs
    if 1 <= num_inputs <= 25:

        # Get the word array and check for palindrome
        len_array = star_input(num_inputs)
        star_print(len_array)
