0: ADDi 0 0
1: 800000
2: CPi 112 0    //BZJi
3: CPi 116 8    //*116=8 jump
4: CPi 88 1     //first_point 
5: CPi 92 0     //*92=0 stepcnt
6: CPi 88 1     //*88=1
7: CPi 96 0     //*96=0 delaycnt 
8: CP 511 88    //*100=*88 ledupdate 
9: ADDi 96 1    //*96=*96+1 delaycnt++
10: CP 104 1    //*104=*1 delay
11: LT 104 96   //if(*104<*96) *104=1 else *104 =0
12: BZJ 116 104 //if(*104==0) goto pc *116=8 else next
13: SRLi 88 33   //SRL
14: CPi 96 0    //delay_reset
15: ADDi 92 1   //stepcnt++
16: CPi 60 7    //seven_step    
17: LT 60 92    //if(*60<*92) *60=1 else *60=0
18: BZJ 116 60  //if(*60==0) goto pc-8 else next
19: CPi 92 0    //stepcnt_reset
20: BZJi 112 6  //end 