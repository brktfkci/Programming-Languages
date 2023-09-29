class HashMap:
    def __init__(self):
        self.hash_map = {}

    def add(self, key, value):
        self.hash_map[key] = value

    def remove(self, key):
        if key in self.hash_map:
            del self.hash_map[key]
        else:
            print(f"Key '{key}' not found in the hash map.")

    def search(self, key):
        return self.hash_map.get(key, None)

# Example usage:
my_map = HashMap()

# Adding items
my_map.add("name", "Alice")
my_map.add("age", 30)
my_map.add("city", "New York")

# Removing items
my_map.remove("age")

# Searching items
print("Search for 'name':", my_map.search("name"))
print("Search for 'gender':", my_map.search("gender"))
