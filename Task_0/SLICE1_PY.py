'''
This script is code stub for CodeChef problem code APLAM1_PY
Filename:      APLAM1_PY_solution.py
Created:       27/09/2021
Last Modified: 27/09/2021
Author:        e-Yantra Team
'''

def perform_operations(n, lis):

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

        n = int(input())
        numbers = list(map(int, input().split()))

        perform_operations(n, numbers)
