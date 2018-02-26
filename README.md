# MagnasineMagPy
A tool for monitoring a Magnum inverter using Python and a RS485 to serial converter. 

### Credits
Chris (aka user cpfl) Midnightsolar forum.
Author: Paul Alting van Geusau
Modifications by: Liam O'Brien

## Backstory
After purchasing a Magnum MS4488PAE inverter to use with my off-grid power setup I wanted a way to monitor and control the system
without buying the over-priced Magnum Remote for $200. After some digging I found [this](http://midniteftp.com/forum/index.php?topic=2458.0) forum topic that covered logging data using a RaspPi
and RS485-Serial adapter. It turns out that the code needed to be modified slightly to work with a remote-less setup without failing. After I fixed that I also added some extra data outputs.
The goal is to integrate this data into an EmonCMS install to do long term logging of my PV array and inverter, we'll see how that goes.


## Hardware and Pinout
As it would happen the pinout described by Magnum's Comms protocal document was out of date, at least for my inverter. I had to spend an evening probing
the output of the RJ-11 jack with a logic analyzer and oscilloscope to find the RS485 lines. Once I did everything worked perfectly.

To save you some time on your dataloggin journey see the pinout below, this is assuiming you go from left to right with the connector clip facing away from you:

Green "Network" port on the MS4488PAE
- 1 = NC
- 2 = (A+ 0-5V)
- 3 = (GND)
- 4 = NC
- 5 = (B- 7-12V)
- 6 = (14V w/respect to 3 as GND)
