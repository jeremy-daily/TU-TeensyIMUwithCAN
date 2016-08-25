#Hardware Design
This folder contains the Altium project, schematic and printed circuit board documents.

If you don't have access to Altium, there is a PDF print.

The board is designed to work with a Teensy 3.1 or Teensy 3.2. (https://www.pjrc.com/store/teensy32.html).

##Errata for Revision 1
### Critical
The specified part fort the CAN tranceiver needs to be changed. The MCP2562 is a 5V supplied chip, and the schematic has 3.3V routed to the supply pin. There are two ways to fix this:
 1. Swap the 5V MCP2562 SOIC-8 chip for a 3.3V compatible CAN tracneiver. Specifically the  Texas Instruments SN65HVD232DR (http://www.mouser.com/search/ProductDetail.aspx?R=0virtualkey0virtualkeySN65HVD232DR) will work. Unsolder the MCP2562 and replace it with the TI chip. Remember to keep the orientation the same. The bar on the TI chip indicates PIN1 and the dot on the MCP2562 chip indicates PIN1.
 2. Unsolder U1 Pin 3 from the pad. Bend the pin up with some tweezers, then connect U1-3 to the Vin connection. However, this will only work if the USB cable is connected since there is no other 5V supply.
### Suggested Improvements

 * The analog supply to the analog high G sensor should be tied to AREF on the the Teensy and isolated with a PI filter. 
 * Add an actual battery charger for a Li-Ion or Li-Po cell (18650).
 * Add another switch input port.
