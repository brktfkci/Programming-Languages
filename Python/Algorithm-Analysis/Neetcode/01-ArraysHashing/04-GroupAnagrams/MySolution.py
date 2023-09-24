class Solution:
    def groupAnagrams(strs):
        
        # list_dict_an = []
        
        # for str_an in strs:
        #     dict_an = {}
        #     for char_an in str_an:
        #         if char_an not in dict_an.keys():
        #             dict_an[char_an] = 1
        #         else:
        #             dict_an[char_an] += 1
        #     list_dict_an.append(tuple(sorted(dict_an.items())))
            
        # dict_res = {}
        
        # for ii in range(len(list_dict_an)):
        #     if list_dict_an[ii] not in dict_res:
        #         dict_res[list_dict_an[ii]] = [ii]
        #     else:
        #         dict_res[list_dict_an[ii]].append(ii)
        
        # list_results = []
        # for values in dict_res.values():
        #     list_result = []
        #     for value in values:
        #         list_result.append(strs[value])
        #     list_results.append(list_result)

        # return list_results
        
        dict_res = {}
        
        for str_val in strs:
            count = [0 for ii in range(26)]
            for char in str_val:
                count[ord(char) - ord("a")] += 1
            if tuple(count) not in dict_res:
                dict_res[tuple(count)] = [str_val]
            else:
                dict_res[tuple(count)].append(str_val)
        
        return [ii for ii  in dict_res.values()]
    
strs = ["eat","tea","tan","ate","nat","bat"]
sol = Solution.groupAnagrams(strs)






