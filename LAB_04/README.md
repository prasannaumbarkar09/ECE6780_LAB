LAB 04 (UART)

PART1-- Blocking Reception

In this part we created a loop which exits when the flag is set and when a key is pressed with matching LED color the LED is turned on,
When a wrong key is pressed an error is printed on console.


PART 2-- Interrupt Based Reception

In this we included interrupt with interrupt function where two inputs are taken 
The first character is a letter matching the LED colors,
If the character doesn't match an error is prinited on console
The second character is a number between 0 and 2.
– ‘0’ turns off the LED
– ‘1’ turns on the LED
– ‘2’ toggles the LED
