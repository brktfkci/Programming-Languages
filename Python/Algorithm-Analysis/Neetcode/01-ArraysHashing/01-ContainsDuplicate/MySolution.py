class Solution:
    def contains_duplicate(self, nums: List[int]) -> bool:
        set_num = set()
        for num in nums:
            if num in set_num:
                return True
            set_num.add(num)
        return False

# adding element to set is O(1)