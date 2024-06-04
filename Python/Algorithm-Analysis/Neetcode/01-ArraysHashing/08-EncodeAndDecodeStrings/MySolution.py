class Solution:

    def encode(strs: list[str]) -> str:
        str_result = ""
        for word in strs:
            str_result = str_result + str(len(word)) + "$"  + word   
        return str_result
                

    def decode(s: str) -> list[str]:
        list_result = []
        ii = 0
        while ii < len(s):
            jj = ii
            while s[jj] != "$":
                jj += 1
            len_sub = int(s[ii:jj])
            list_result.append(s[jj+1:jj+1+len_sub])
            ii = jj+1+len_sub
        return list_result
            
            
inpt = ["leet","code","love","you"]
enc_sol = Solution.encode(inpt)
dec_sol = Solution.decode(enc_sol)
print(enc_sol)
print(dec_sol)
