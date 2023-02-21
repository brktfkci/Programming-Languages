In order to communicate FPGA with the RS232 port of the computer, a client program like Hyperterminal or Putty should be used. Communication protocols can be seen below;
('#' stands for hex numbers)
A###      -> sets the address to "###".
W######## -> writes '########' to current address.
R         -> reads data from current address. (Output will be "R########")
S         -> resets MPU (PC is reset).
F         -> pauses MPU (PC is not reset).
C         -> continues MPU (from current PC).
I         -> increments program flow (branching considered).
G###      -> continues MPU till address "###".

Sample codes are available in the current folder.
	"freezeTest.txt" can be used to get familiar with the communication protocol.
	"sort.txt" sorts the numbers from addr 100 to 110.

COM Port Settings:

	bits per seconds -> 115200
	data bits        -> 8
	parity           -> None
	stop bits        -> 1
	flow control     -> None

Some settings that are advisable:
	Hyperterminal -> properties -> settings -> ASCII Setup -> Send line ends with line feeds          (selected)
	                                                       -> echo typed characters locally           (selected)
	                                                       -> append line feeds to incoming line ends (selected)
	                                                       -> wrap lines that exceed terminal width   (selected)