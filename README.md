phoenixpilot
========================
phoenixpilot is a L2 autonomous driving system for Ford vehicles. 

The upstream code base for this repository can be found at https://github.com/commaai/openpilot

PHOENIXPILOT IS NOT ENDORSED, AFFILIATED, OR ASSOCIATED WITH COMMA OR OPENPILOT. 

This fork implements steer-by-wire with the use of the Active Park Assist system. 
This requires the PSCM to be speed spoofed, and as such, the usual torque maps are not present. This can cause a floating feel on the steering wheel, but you are in complete control of the steering. 

This fork also will not work with stock hardware, and requires additional hardware to be used with it. 

This software is development grade, and should not be used on public roads. Use at your own risk. 