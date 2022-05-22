# A more sophisitcated FM receiver

I forked this from https://github.com/JvanKatwijk/sdr-j-fm, thanks Jan van Katwijk for this great work. 

I went on my work based on V2.0 (SHA1: 8e917ac).

For setup procedure see the original link above.

I only could go on with this work under Linux (Xubuntu), I did not built any Windows packages yet.

As I only have a DabStick (RTLSDR) for testing, I build-in an autostart only for this device. Possibly, this has to be deactivated if you use another device:
Remove the line
  `QTimer::singleShot(100, this, [this](){ setDevice("dabstick"); } );`
in radio.cpp for this.

## What is new?
* Change color design of the spectrum for better readability (only a subjective thing, of course)
* Remove the vertical red line for the left base band spectrum as it delivers no extra information and hid the DC peak.
* Make the MAX and AVG numbers in the spectrum views colored and give them names.
* More meaningful text on the Squelch button whether the squelch is currently on or off.
* Added a unit description to several comboboxes with numbers and reorder them to a more suitable place.
* The combobox of the used demodulator type shows now the demodulator name directly. The extra text box used for this formerly is still there.
* There is now a "Stereo panorama" slider which can move the audio to pure Mono (left), Stereo (mid) to enhanced Stereo (right). This slide was intended to lower the noise with weak signals (the more "Mono" the less noise). Maybe there will be an automatic mode in future for this as it car FM radios provides.
* The pilot indicator is now not anymore derived from the pilot level but on the lock-state of the pilot-PLL. There was also an improvement of the PLL-lock detection necessary (only experimental) as it worked not reliable (at least for my device).
* The pilot detection is used to automatically switch to Mono-mode if the pilot is not detected. This behavior is selectable in the GUI with the checkbox "Auto mono". 
* When saving a frequency the RDS station name is used as a proposal.
* I removed the automatic audio level control as its sense how this worked was not clear to me and it took too long till it was loud enough.
* There is an AFC (Automatic Frequency Control) functionality now with a switchable GUI element. It moves the frequency automatically to the center of the signal. It works fine so far but could maybe faster as a analog radio would do.
* I increased the filter tap of the AF filter from 11 to 55 as with 11 there was no hearable effect (the 55 are also choosen by experiment)
* The balance slider makes only one audio channel quiter but keeps the other channels loudness. So no audio overdriven may happen.
* Instead of showing the version number the last commit SHA1 (first seven digits) is shown in the GUI.
* And finally I did many code refinings to be better suitable to my kind of thinking and better understanding the code :-).


## How is it looking now?
![fm receiver](/fmreceiver.png?raw=true)

