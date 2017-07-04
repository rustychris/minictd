1. Plug micro USB into teensy.  If the teensy is brand new, it should
   be programmed with a blinking program, and the LED will blink maybe
   at 1 Hz.  If it's not brand new, make sure you can communicate with
   it in the Arduino GUI.
2. Solder the crystal and the flying leads on the bottom.  lay the
   crystal down on the board before solder to make the leads are long
   enough.  For the flying A12/A13 leads, bend a short 90, and have
   the wires come off the board on the sides of the pads nearest each
   other
3. File down the tabs on the side of the board from panelization
4. Layout the header pins, long side into breadboard, to match the
   holes with the *top* side of the pcb face up.  The short side of
   the pins will go into the logger pcb, and the long side into the
   teensy holes.  Don't forget the two header pins on the inner row.
   Solder the pcb to the headers.
5. Now the pcb is firmly held by the breadboard, and some smd parts
   can be added.  Start with a few resistors or caps which far from
   other components to get in the mood.
6. AD623 and MCP6072
   [my U7 was a slightly different package - still marked AD623A, but
    second line was #1705, while the other two were #1711]
7. The LD3985 is the smallest - now that you've got a good groove
   going with the SOICs, tackle the LD3985.
   [footprint could be a bit more generous, or maybe I just got it
    started off center]
[did I leave 22.1k in there in place of something purports to be 50?
 yep - both for the voltage divider, and for the battery sense oops.]    
[probably should have spec'd some 2k resistors for the gain, at least
on the current sense.  current sense is now on 10ohm, instead of 500,
so signal is down a factor of 50.  was gain of 10, so in theory
a gain of 500 would be good.  that wouldr require a 200ohm.  okay -
go with a 470, since I already have one.]
8. Go through the rest of the resistors, capacitors, the transformer.
9. Flip the board over, and solder in pin headers for the RTD, I2c to
   pressure sensor, battery connection.
10. Solder on the sd card lost
11. Last thing is to solder on the teensy, threading the flying leads
    through the PCB to soldered from the other side.
    
