MPU reads the code from the block memory inside the FPGA. Machine code is entered to the block ram via Hyperterminal.
Machine code is entered in hex (0123456789abcdef). The other significant letters are:

'A'ddress   A064      goes to address 100 in decimal.
'W'rite     W12345abc writes 12345abc to the current address and increments the address.
'R'ead      R         reads the current address.
’S’tart/top S         starts/stops the code execution.
'F'reeze    F         pauses the code execution.
'C'ontinue  C         continues the code execution.
'G'o to     G064      executes the code till address 100 in decimal.
'I'ncrement I         increments the flow by one.

More details about Hyperterminal can be found in the "readme_terminal.txt" file.