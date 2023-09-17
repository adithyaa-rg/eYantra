''''
# Team ID:          eYRC#HB#2008
# Theme:            Hologlyph Bots
# Author List:      Haricharan B
# Filename:         INV_PY.py
# Functions:        Inventory.__init__, Inventory.add, Inventory.delete
# Global variables: test_cases, n, items_dictionary, item_name, quantity, inventory, m, type_name, item_name, quantity
'''
class Inventory:
    """
    Purpose:
    ---
    This class is used to store the inventory of a shop

    Attributes:
    ---
    inventory (dict): The dictionary containing the inventory

    Methods:
    ---
    add(item_name, value): Adds the item to the inventory
    delete(item_name, value): Deletes the item from the inventory
    """

    def __init__(self, inventory = {}):
        """
        Purpose:
        ---
        The constructor for Inventory class

        Input Arguments:
        ---
        inventory (dict): The dictionary containing the inventory

        Returns:
        ---
        None

        Example call:
        ---
        Inventory({'A' : 10, 'B' : 15})
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
