0: ADDi 0 0
1: 160000
2: CPi 80 26     //BZJ
3: CPi 84 1     //initial flag=1 
4: CPi 112 0    //BZJi
5: CPi 116 10   //*116=10 jump
6: CPi 88 1     //first_point 
7: CPi 92 0     //*92=0 stepcnt
8: CPi 88 1     //*88=1
9: CPi 96 0     //*96=0 delaycnt 
10: CP 511 88   //*100=*88 ledupdate 
11: ADDi 96 1   //*96=*96+1 delaycnt++
12: CP 104 1    //*104=*1 delay
13: LT 104 96   //if(*104<*96) *104=1 else *104 =0
14: BZJ 116 104 //if(*104==0) goto pc *116=10 else next
15: BZJ 80 84   //if(flag==0) goto pc-26 else next
16: SRLi 88 33  //SRL
17: CPi 96 0    //delay_reset
18: ADDi 92 1   //stepcnt++
19: CPi 60 7    //seven_step    
20: LT 60 92    //if(*60<*92) *60=1 else *60=0
21: BZJ 116 60  //if(*60==0) goto pc-10 else next
22: CPi 92 0    //stepcnt_reset 
23: SRLi 88 2   //shift_right 
24: CPi 84 0    //flag=0
25: BZJi 112 10 //end 
26: SRLi 88 1   //right shift
27: CPi 96 0    //delay_reset
28: ADDi 92 1   //stepcnt++
29: CPi 60 5    //seven_step
30: LT 60 92    //if(*60<*92) *60=1 else *60=0
31: BZJ 116 60  //if(*60==0) goto pc-10 else next 
32: CPi 92 0    //stepcnt_reset 
33: CPi 84 1    //flag=1
34: BZJi 112 10  