# kOS-RTLS-Landing

A kOS program that can precisely land a booster, SpaceX style. This is a WIP version and still needs quite a lot of work.

I recommend downloading this craft file: https://kerbalx.com/Patrykz94/Falcon-9

1. Place all of the files in your Ships/Script folder in KSP install directory.

2. Make sure that the kOS CPU on the booster has the boot file set to "boot_falcon".

3. The landing script should work on most launch programs (or even if you launch rocket manually), as long as they/you stage once the liquid fuel is cut. The booster will turn fuel flow back on after 1 second and it will start the boostback and landing procedure. If you would like to use the program provided for launching, I recommend setting it ("boot_testflight") as a boot file on stage 2 CPU. Alternatively, you can just run it manually on the pad.

4. If you use the "boot_testflight" program to launch, the program will wait until you press the Action Group 1 key, and then it will start the 10 second countdown. If you want to change that, or the launch parameters, please edit the "boot_testflight" file.
