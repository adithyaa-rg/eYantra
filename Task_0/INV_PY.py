'''
This script is code stub for CodeChef problem code APLAM1_PY
Filename:      APLAM1_PY_solution.py
Created:       27/09/2021
Last Modified: 27/09/2021
Author:        e-Yantra Team
'''

class Inventory:

    def __init__(self, inventory = {}):
        self.inventory = inventory

    def add(self, item_name, value):

        if (item_name in self.inventory):
            self.inventory[item_name] += value
            print(f"UPDATED Item {item_name}")
        else:
            self.inventory[item_name] = value
            print(f"ADDED Item {item_name}")

    def delete(self, item_name, value):

        if (item_name in self.inventory):

            if (self.inventory[item_name] < value):
                print(f"Item {item_name} could not be deleted")
            else:
                self.inventory[item_name] -= value
                print(f"DELETED Item {item_name}")
        else:
            print(f"Item {item_name} does not exist")

# Main function
if __name__ == '__main__':
    
    # take the T (test_cases) input
    test_cases = int(input())

    for _ in range(test_cases):    

        n = int(input())
        dictionary = {}

        for _ in range(n):

            key, value = input().split()
            dictionary[key] = int(value)
        
        inventory = Inventory(dictionary)

        m = int(input())

        for _ in range(m):

            type_name, key, value = input().split()
            value = int(value)

            if (type_name == "ADD"):
                inventory.add(key, value)

            elif (type_name == "DELETE"):
                inventory.delete(key, value)

        print(f"Total Items in Inventory: {(sum(inventory.inventory.values()))}")
