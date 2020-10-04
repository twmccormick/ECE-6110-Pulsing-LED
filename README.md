# ECE 6110 Pulsing LED

 ECE 6110 - Tyler McCormick - Assignment 1

 This program: initializes LED_2 as an alternate function GPIO 
 and assigns it the ch2n output of timer 1, initilizes timer 1 
 with an ARR of 3999 and a PSC of 199, which creates a period of 10ms,
 and then sets the duty cycle to 50% 
 and then in a continous loop, slowly alters the duty cycle from 
 0% to 100% and then from 100% to 0%.
