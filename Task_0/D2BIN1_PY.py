''''
# Team ID:          eYRC#HB#2008
# Theme:            Hologlyph Bots
# Author List:      Tripan Dham
# Filename:         D2BIN1_PY.py
# Functions:        dec_to_binary
# Global variables: test_cases, n, bin_num
'''

# Function to convert decimal number to binary
def dec_to_binary(n):
    '''
    Purpose:
    ---
    Function to convert decimal number to binary

    Input Arguments:
    ---
    `n` :  [ int ]
        Input integer to be converted to binary

    Returns:
    ---
    `n%2 + dec_to_binary(int(n//2))*10` :  [ int ]
        Returns the binary of the given decimal number in integer datatype

    Example call:
    ---
    binary = dec_to_binary(10)
    '''
    if (n == 0):
        return 0

    return n%2 + dec_to_binary(int(n//2))*10


# Main function
if __name__ == '__main__':
    
    # take the T (test_cases) input
    test_cases = int(input())

    # Write the code here to take the n value
    for case in range(1,test_cases+1):
        # take the n input values
        n = int(input())

        # print (n)

        # Once you have the n value, call the dec_to_binary function to find the binary equivalent of 'n' in 8-bit format
        bin_num = dec_to_binary(n)
        print(str(bin_num).zfill(8))
