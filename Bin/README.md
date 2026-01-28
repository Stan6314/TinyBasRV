Notice!
=======

Be careful when reading and downloading HEX files from GitHub. When downloading a file, the end of the line is truncated to a single LF character (Linux-style). However, WCH-LinkUtility requires Windows-style line termination - CR+LF! When loading a file into its buffer, WCH-LinkUtility will only load one line of HEX file!

WCH-LinkUtility then acts as if it programmed the entire memory, but only 16 bytes are written. That is why the HEX files are now saved in .zip format.

* * *

Upozornění!
===========

Dejte si pozor na čtení a stahování HEX souborů z GitHub. Při stažení souboru je konec řádku zkrácen na jeden znak LF (jako v Linuxu). WCH-LinkUtility ale vyžaduje ukončení řádku ve stylu Windows - CR+LF! Při načítání souboru do svého bufferu pak WCH-LinkUtility načte pouze jeden řádek HEX souboru!

WCH-LinkUtility pak fungují jakoby naprogramovaly celou paměť, ale zapíše se pouze 16 bajtů. Proto jsou HEX soubory nyní uloženy v .zip formátu.