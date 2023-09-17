''''
# Team ID:          eYRC#HB#2008
# Theme:            Hologlyph Bots
# Author List:      Haricharan B
# Filename:         WLEN_PY.py
# Functions:        count_words
# Global variables: test_cases, string
'''
 
# Function to count number of words
def count_words(string):
    """
    Purpose:
    ---
    This function counts the number of words in a string

    Input Arguments:
    ---
    string (str): The string to be counted

    Returns:
    ---
    None

    Example call:
    ---
    count_words("Hi")
    """

    words = string[1:].split()
    lengths = map(len, words)

    print(*lengths, sep = ",")

# Main function
if __name__ == '__main__':
    
    # Take the T (test_cases) input
    test_cases = int(input())

    for i in range(test_cases):

        # Taking input of string
        string = sys.stdin.readline()
        count_words(string)