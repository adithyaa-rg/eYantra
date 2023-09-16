'''
This script is code stub for CodeChef problem code APLAM1_PY
Filename:      APLAM1_PY_solution.py
Created:       27/09/2021
Last Modified: 27/09/2021
Author:        e-Yantra Team
'''

# Function to generate the A.P. series
def find_maximum(n, dictionary):

    maximum = max(dictionary.values())

    students_list = []

    for key, value in dictionary.items():
        if value == maximum:
            students_list.append(key)

    sorted(students_list)
    print(*students_list, sep = "\n")

# Main function
if __name__ == '__main__':
    
    # take the T (test_cases) input
    test_cases = int(input())

    for _ in range(test_cases):    

        n = int(input())
        dictionary = {}

        for _ in range(n):

            key, value = input().split()
            dictionary[key] = float(value)
        
        find_maximum(n, dictionary)
