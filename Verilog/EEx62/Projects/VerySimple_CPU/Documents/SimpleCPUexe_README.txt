NOTES:
* memgen and SimpleCPU are combined.

HOWTO:
> SimpleCPU.exe {input}
Parses and creates memin.txt, memory.v, mem232.txt
Then asks whether to run the code or quit.

> SimpleCPU.exe {input} r
Parses and creates memin.txt, memory.v, mem232.txt.
Runs the code without asking.

> SimpleCPU.exe {input} q
Parses and creates memin.txt, memory.v, mem232.txt.
Quits after creating the files without asking.

During the parsing stage, mid.txt is created. It contains the code with memory address numbers inserted.


After a clean exit the program generates memoutd.txt and memouth.txt. moutd.txt contains the memory contents in decimal. memouth.txt contains the memory contents in hex.


Features implemented between Jun - Nov 2012:

- If there is a parse error, the program line causing it gets printed.

- Lines with no address specified have the address of the previous line plus one. Preprocessor figures out addresses and creates a file called mid.txt with all line addresses in place. Then the simulator actually runs this program file.

- Simulator exits if the address PC points to is 0

- Simulator falls into the prompt mode if PC does not change after an instruction eceutes.

- In prompt mode:
  - "exit" -> exits the simulator and dumps the current memory to memout.txt
  - "A: B" -> where A and B are unsigned numbers, writes B into address A
  - "A: 0xB" -> where A and B are unsigned integers, writes 0xB (hex) into address A
