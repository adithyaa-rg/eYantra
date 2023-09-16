'''
This script is code stub for CodeChef problem code APLAM1_PY
Filename:      APLAM1_PY_solution.py
Created:       27/09/2021
Last Modified: 27/09/2021
Author:        e-Yantra Team
'''

# Function to print the maximum marks students
def find_maximum(n, marks_dictionary):
    """
    This function prints the students with maximum marks

    Parameters:
    n (int): The number of students

    Returns:
    None
    """

    maximum = max(marks_dictionary.values())

    students_list = []

    for student, marks in marks_dictionary.items():
        if marks == maximum:
            students_list.append(student)

    sorted(students_list)
    print(*students_list, sep = "\n")

# Main function
if __name__ == '__main__':
    
    # take the T (test_cases) input
    test_cases = int(input())

    for _ in range(test_cases):    

        # Taking input of n
        n = int(input())
        marks_dictionary = {}

        # Taking input of key and value
        for _ in range(n):
            student, marks = input().split()
            marks_dictionary[student] = float(marks)
        
        find_maximum(n, marks_dictionary)
