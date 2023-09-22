# Two Sum

Given an array of integers `nums` and an integer `target`.

Return indicies of two numbers such that they add up to `target`.

> * You may assume that each input would have **_exactly one solution_**, and you may not use the same element twice. 
> * You can return the answer in any order.

**Example 1:**
<pre>
<b>Input:</b> nums = [2,7,11,15], target = 9 
<b>Output:</b> [0,1]
<b>Explanation:</b> Because, nums[0] + nums[1] == 9, we return [0,1]
</pre>

**Example 2:**
<pre>
<b>Input:</b> nums = [3,2,4], target = 6 
<b>Output:</b> [1,2]
</pre>

**Example 3:**
<pre>
<b>Input:</b> nums = [3,3], target = 6 
<b>Output:</b> [0,1]
</pre>

**Constraints:**

* $2 \leq nums.length \leq 10^4$
* $-10^9 \leq nums[i] \leq 10^9$
* $-10^9 \leq target \leq 10^9$
*  __Only one valid answer exists.__

**Follow-up:** Can you come up with an algorithm that is less than $0(n^2)$ time complexity?


