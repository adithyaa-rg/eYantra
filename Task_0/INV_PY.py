'''
This script is code stub for CodeChef problem code APLAM1_PY
Filename:      APLAM1_PY_solution.py
Created:       27/09/2021
Last Modified: 27/09/2021
Author:        e-Yantra Team
'''

class Inventory:
    """
    This class is used to store the inventory of a shop

    Attributes:
    inventory (dict): The dictionary containing the inventory

    Methods:
    add(item_name, value): Adds the item to the inventory
    delete(item_name, value): Deletes the item from the inventory
    """

    def __init__(self, inventory = {}):
        """
        The constructor for Inventory class

        Parameters:
        inventory (dict): The dictionary containing the inventory

        Returns:
        None
        """

        self.inventory = inventory

    def add(self, item_name, quantity):
        

        if (item_name in self.inventory):
            self.inventory[item_name] += quantity
            print(f"UPDATED Item {item_name}")
        else:
            self.inventory[item_name] = quantity
            print(f"ADDED Item {item_name}")

    def delete(self, item_name, quantity):

        if (item_name in self.inventory):

            if (self.inventory[item_name] < quantity):
                print(f"Item {item_name} could not be deleted")
            else:
                self.inventory[item_name] -= quantity
                print(f"DELETED Item {item_name}")
        else:
            print(f"Item {item_name} does not exist")

# Main function
if __name__ == '__main__':
    
    # take the T (test_cases) input
    test_cases = int(input())

    for _ in range(test_cases):    

        n = int(input())
        items_dictionary = {}

        for _ in range(n):

            item_name, quantity = input().split()
            items_dictionary[item_name] = int(quantity)
        
        inventory = Inventory(items_dictionary)

        m = int(input())

        for _ in range(m):

            type_name, item_name, quantity = input().split()
            quantity = int(quantity)

            if (type_name == "ADD"):
                inventory.add(item_name, quantity)

            elif (type_name == "DELETE"):
                inventory.delete(item_name, quantity)

        print(f"Total Items in Inventory: {(sum(inventory.inventory.values()))}")
