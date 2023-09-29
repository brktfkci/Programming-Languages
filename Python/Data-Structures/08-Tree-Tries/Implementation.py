class TrieNode:
    def __init__(self):
        self.children = {}
        self.is_end_of_word = False

class Trie:
    def __init__(self):
        self.root = TrieNode()

    def insert(self, word):
        node = self.root
        for char in word:
            if char not in node.children:
                node.children[char] = TrieNode()
            node = node.children[char]
        node.is_end_of_word = True

    def search(self, word):
        node = self.root
        for char in word:
            if char not in node.children:
                return False
            node = node.children[char]
        return node.is_end_of_word

    def remove(self, word):
        def _remove_recursive(node, word, depth):
            if depth == len(word):
                if node.is_end_of_word:
                    node.is_end_of_word = False
                    return not bool(node.children)
                return False
            char = word[depth]
            if char not in node.children:
                return False
            should_delete = _remove_recursive(node.children[char], word, depth + 1)
            if should_delete:
                del node.children[char]
                return not bool(node.children)
            return False

        _remove_recursive(self.root, word, 0)

# Example usage:
my_trie = Trie()

# Inserting words
my_trie.insert("apple")
my_trie.insert("app")
my_trie.insert("banana")

# Searching for words
print("Search for 'apple':", my_trie.search("apple"))
print("Search for 'apples':", my_trie.search("apples"))

# Removing words
my_trie.remove("app")
print("Search for 'app':", my_trie.search("app"))
