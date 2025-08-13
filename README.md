spent the last ~5 days learning about and hacking on this FLIP/PIC fluid simulation.

code is in /src/main.rs. will be organizing and improving in the coming days.

**a quick gif of my finished progress at the end of this readme**

wanted to see if i could get a reasonable version up and running quickly on my microbit to learn more about rust, embedded development, and applied software projects.

i'm happy i got to brush up on some math, rack my brain a bit, and then pull an all nighter to get this done quicker. i'm even more excited to continue working on it - and potentially make a browser version as intended.

i closely followed this tutorial by 10 minute physics and tried to avoid looking at the provided source code as much as possible. 

notably, some things i'll continue to work on for the next few days to polish it up:
- lights are currently blinking - somethign to do with how i've set up setting and resetting the led array on each step. 
- i want more of a fluid 'stacking' visual - i suspect it may have to do with how i'm dealing with particle-particle collisions and enforcing the boundaries
- playing with the simulation parameters to see if any net a more realistic looking simulation. i'm most bottlenecked by the 5x5 led array, but i think i can do slightly better than this.

![gif](flip_fluid_demo.gif)