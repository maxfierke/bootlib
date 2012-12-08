====================
Description:
====================
bootlib is a library that aids in the booting of .nds files in programs.

It includes:
GBAMP Internal Loader
SuperCard Internal Loader
EZ-Flash IV (4) Internal Loader
Chishm's DLDI Loader
M3 Internal Loader
====================
Copyrights:
====================
bootlib 1.3b
Copyright 2007 Max Fierke (TeenDev)
load.bin
Copyright 2007 Michael Chisholm (chishm)
====================
License:
====================
GPLv3
====================
Requirements:
====================
* DevKitARM r20/r21

* libfat

* libnds or PAlib
====================
Usage:
====================
* Refer to the included example for your prefered library (libnds or PAlib).
* Include the bootlib directory in your project root (for the example project)

ARM9:
void bootnds(filename);

ARM7:
void bootndsCheck();

* Just call the above functions for each processor. The library features automatic cart detection and calls a different loader for each flashcart.

* Add the *.bin into your arm9 "data" directory.
====================
THANKS:
====================
* Lick (Looking at the rebootlib source helped me make a structure for this library.)
* Chishm (For the Generic NDS Loader as well as the GBAMP one too. This would not be possible without libfat either)
* Moonlight (For the SuperCard Loader and EZ 4 Loader)
* DragonMinded (For the SuperCard Loader)
