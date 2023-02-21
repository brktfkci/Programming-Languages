class ListNode(object):
    def __init__(self, val=0, nextNode=None):
        self.val = val
        self.nextNode = nextNode

class Solution(object):
            
    def addTwoNumbers(self, l1: ListNode, l2: ListNode) -> ListNode:
        """
        :type l1: ListNode
        :type l2: ListNode
        :rtype: ListNode
        """
        ListNodeResult = ListNode()
        ListNodeCurr = ListNodeResult
        
        carry = 0
        while l1 or l2 or carry:
            val1 = l1.val if l1 else 0
            val2 = l2.val if l2 else 0
            
            #new digit
            valNext = val1 + val2 + carry
            carry = valNext // 10
            valNext = valNext % 10
            ListNodeCurr.nextNode = ListNode(valNext)
            
            #update pointers
            ListNodeCurr = ListNodeCurr.nextNode
            l1 = l1.nextNode if l1 else None
            l2 = l2.nextNode if l2 else None
            
        return ListNodeResult.nextNode
    
    

            
            
            
            
            
            
            