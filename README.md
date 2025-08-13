spent the last ~5 days learning about and hacking on this FLIP/PIC fluid simulation.

code is in /src/main.rs. will be organizing and improving in the coming days.

**a quick gif of my finished progress at the end of this readme**

wanted to see if i could get a reasonable version up and running quickly on my microbit to learn more about rust, embedded development, and applied software projects.

i'm happy i got to brush up on some math, rack my brain a bit, and then pull an all nighter to get this done quicker. i'm even more excited to continue working on it - and potentially make a browser version as intended.

i closely followed this [tutorial by 10 Minute Physics](https://www.youtube.com/watch?v=XmzBREkK8kY) and tried to avoid looking at the provided source code as much as possible. massive thanks to 10 Minute Physics for the guidance + help.

notably, some things i'll continue to work on for the next few days to polish it up:
- lights are currently blinking - certainly with how I have set up resetting and setting the led array. This will be an easy fix.
- i want more of a fluid 'stacking' visual. i suspect it may have to do with how particle-particle collisions and boundaries have been implemented. tweaking these will definitely lend better results.
- playing with the simulation parameters to see if i can net a more realistic looking simulation. i'm most bottlenecked by the 5x5 led array (it's a low def visual), but believe i can do slightly better than this.

![gif](IMG_4149.gif)