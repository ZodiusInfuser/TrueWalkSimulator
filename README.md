# TrueWalk-Unity
TrueWalk is a motion planning and gait generation system for controlling virtual and real-world multi-legged robots. This repository is for the Unity version of the system (currently the only version).

## About
This presents my latest efforts to create a walking algorithm that is both powerful enough to produce dynamic motion of statically-stable walking robots and light-weight enough to be portable over to real hardware in the future.

Here is a demo of a Decapod walking in a basic Unity scene
[![Video of a simulated decapod walking](https://i.imgur.com/ezKssWB.png)](https://www.youtube.com/watch?v=jciutQUr8NA)

## Background
My initial exploration of walking algorithms was around 15 years ago when I created the TrueWalkScript for the video game Total Annihilation. In that game "bots" play animations as they move around to give the illusion of walking. This was fine for the base game but as the modding community created ever bigger bots this illusion broke down, requiring a more advanced system that would accurately position the legs whilst the bot moved, but also be able to handle sudden direction changes. Below are two examples from the game that use the TrueWalkScript.

![Animation of Zarganeth bot from TA walking](https://i.imgur.com/VkB4RCT.gif)
![Animation of CORE MKL bot from TA walking](https://i.imgur.com/eW0Ifp1.gif)

(These gifs were created by and used with the permission of [ArmouredFish](https://www.tauniverse.com/forum/member.php?u=5200) to demonstrate their [Robot War Engine](https://www.tauniverse.com/forum/showthread.php?t=45555)).

Since this script I have explored walking algorithms in different settings, such as a C# XNA simulator I wrote 10 years ago, and a custom arduino-based Hexapod walker I built 5 years ago.

[![Video of a simulated hexapod walking](https://i.imgur.com/TVnsxvj.png)](https://www.youtube.com/watch?v=lgRSC-BZ5_g)

[![Video of a real hexapod walking](https://i.imgur.com/7pARIKC.png)](https://www.youtube.com/watch?v=sR4aj7tOwko)
