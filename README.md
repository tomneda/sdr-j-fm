# A more sophisticated FM receiver

I forked this from https://github.com/JvanKatwijk/sdr-j-fm, thanks Jan van Katwijk for this great work. 

I went on my work based on V2.0 (SHA1: 8e917ac).

For Linux setup procedure see the original link above.

I could also setup a build system for Windows. I try to describe the setup procedure later here.

As I only have a DabStick (RTLSDR) for testing, I build-in an autostart only for this device. Possibly, this has to be deactivated if you use another device:
Remove the line
  `QTimer::singleShot(100, this, [this](){ setDevice("dabstick"); } );`
in radio.cpp for this.

## How it is looking now?
![fm receiver](/fmreceiver.png?raw=true)

## What is new relative to the original sdr-j-fm (V2.0)?
* Massively changes in RDS processing to improve sensitivity: Avoiding the usage of the 19 kHz pilot the RDS is decoded with "classic BPSK decoding style" (rough frequency estimation, time sychronisation with M&M (Mueller and Muller), fine frequency correction with Costas loop). Finally the RDS BPSK signal is shown in an extra IQ plot. Disadvantage of this change: more processor power is needed. Switching off RDS avoid using this processing power.
* Change color design of the spectrum for better readability (only a subjective thing, of course)
* A stylesheet "Combinear" from https://qss-stock.devsecstudio.com/ is used.
* Remove the vertical red line for the left base band spectrum as it delivers no extra information and hid the DC peak.
* Make the MAX and AVG numbers in the spectrum views colored and give them names.
* Show only a single side MUX spectrum now, as there was no special information with a real-valued double sided spectrum. Also is the frequency resolution doubled now. 
* The averaging of the spectrum displays are reset if the frequency has changed for a better fluently adapted display content.
* More meaningful text on the Squelch button whether the squelch is currently on or off.
* Added a unit description to several comboboxes with numbers and reorder them to a more suitable place.
* The combobox of the used demodulator type shows now the demodulator name directly. The extra text box used for this formerly is still there.
* There is now a "Stereo panorama" slider which can move the audio to pure Mono (left), Stereo (mid) to enhanced Stereo (right). This slide was intended to lower the noise with weak signals (the more "Mono" the less noise). Maybe there will be an automatic mode in future for this as it car FM radios provides.
* The pilot indicator is now not anymore derived from the pilot level but on the lock-state of the pilot-PLL. There was also an improvement of the PLL-lock detection necessary (only experimental) as it worked not reliable (at least for my device).
* The pilot detection is used to automatically switch to Mono-mode if the pilot is not detected. This behavior is selectable in the GUI with the checkbox "Auto mono". 
* When saving a frequency the RDS station name is used as a proposal.
* I removed the automatic audio level control as its sense how this worked was not clear to me and it took too long till it was loud enough.
* There are two damped peak level meters (above left channel, below right channel) which show the fullscale level (dBFS) of the audio output. The delay of audio buffer related to this peak meters can be compensated with a setting below. A "Cal Test Tone" helps establish this delay.
* A new volume slider can control the audio volume in 0.5dB steps (with -30dB .. 6dB gain). At most left position the audio is fully muted.
* If the audio volume reaches more than 0dBFS then the volume slider is automatically shifted to a lower volume level.
* There is an AFC (Automatic Frequency Control) functionality now with a switchable GUI element. It moves the frequency automatically to the center of the signal. It works fine so far but could maybe faster as a analog radio would do.
* The balance slider makes only one audio channel quiter but keeps the other channels loudness. So no audio overdriven may happen.
* Instead of showing the version number the last commit SHA1 (first seven digits) is shown in the GUI.
* There is an indicator added where the relative miss-tune of the carrier center can be seen.
* Added an AM demodulator with smaller IF/AF filters for AM receptions. With the smaller filters also a NBFM reception is possible.
* The AF filter uses FFT filter with more filter taps to improve the filter performance.
* RDS provides PTY text (Program Type).
* And finally I did many code refinings to be better suitable to my kind of thinking and better understanding the code :-).
