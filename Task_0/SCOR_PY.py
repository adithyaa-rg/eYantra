''''
# Team ID:          eYRC#HB#2008
# Theme:            Hologlyph Bots
# Author List:      Haricharan B
# Filename:         SCOR_PY.py
# Functions:        find_maximum
# Global variables: test_cases, n, marks_dictionary, student, marks
'''

# Function to print the maximum marks students
def find_maximum(n, marks_dictionary):
    """
    Purpose:
    ---
    This function prints the students with maximum marks

    Input Arguments:
    ---
    n (int): The number of students
    marks_dictionary: Dictionary of marks

    Returns:
    ---
    None

    Example call:
    ---
    find_maximum(5, {}"A" : 10, "B" : 20.5})
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
