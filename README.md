# floatimu
*floating point imu code for bradwii*

**THIS CODE MAY CONTAIN BUGS**

it has not been used in modes other then level mode

This is a replacement of bradwii's imu code for hubsan and other quads
It can be cut and pasted or simply replace the original file.


This was a small educational project to see if the truncation errors 
in the fixed point code of the original imu code cause instability.


The accuracy of the code is improved compared to the original, and this 
translates in better flight characteristics.


If it is too big to compile:
*in Kiel uVision Options for target -> use microlib
*in Kiel uVision Options for target -> use crossmodule optimization
*use different approximations by setting SMALL_ANGLE_APPROX and associated options
  

