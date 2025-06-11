We are using Igus' iRC control system for the gantry.

Igus offer various sample programme interfaces for iRC such as LabVIEW, ROS, Modbus and other PLC interfaces such as G-CODE. You can see all information on the download section here: https://www.igus.co.uk/info/programs-igus-robot-control

There is also an incredibly comprehensive Wiki page dedicated to iRC, including a section where you can see how to interface with the robots using the various languages: https://wiki.cpr-robots.com/index.php/CPR_Wiki
-> iRC can be used for simulation and testing of the gantry (also used for connection to the real robot). I think we need a Windows machine for iRC -> need to look into if this will be needed for the final show, or just for simulation testing. -> CRI-Python-Lib looks good for writing control code.
Raspberry Pi (type?) connected to cameras for images of the audience. -> gRPC for image streaming between Raspberry Pi and control computer?