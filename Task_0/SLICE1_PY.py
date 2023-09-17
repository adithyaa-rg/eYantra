''''
# Team ID:          eYRC#HB#2008
# Theme:            Hologlyph Bots
# Author List:      Haricharan B
# Filename:         SLICE1_PY.py
# Functions:        perform_operations
# Global variables: test_cases, n, numbers
'''
 
def perform_operations(n, lis):
    """
    Purpose:
    ---
    This function performs the required operations as specified in the question

    Input Arguments:
    ---
    n (int): The number of elements in the list
    lis (list): The list of numbers

    Returns:
    ---
    None

    Example call:
    ---
    perform_operations(5, [1, 2, 3, 4, 5, 6, 7, 8])
    """

    list_1 = reversed(lis)
    list_2 = map (lambda x: x + 3, lis[3::3])
    list_3 = map (lambda x: x - 7, lis[5::5])
    sum_lis = sum(lis[3:8])

    print(*list_1, sep = " ")
    print(*list_2, sep = " ")
    print(*list_3, sep = " ")
    print(sum_lis)


# Main function
if __name__ == '__main__':
    
    # take the T (test_cases) input
    test_cases = int(input())

    for _ in range(test_cases):    

        # Taking input of n
        n = int(input())

        # Taking input of the list
        numbers = list(map(int, input().split()))

        # Calling the function
        perform_operations(n, numbers)
