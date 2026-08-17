# Notes on how we tune the inductive heater

The inductive heater drive needs to be tuned to ensure the maximum power is delivered and the mosfet and TVS diode remain cool. The following parameters need to be tuned (with a tool in the coil):

* Mosfet ON time for first cycle of the burst. Long enough for the peak mosfet voltage to just reach the target.
* Mosfet OFF time. Very slightly longer than the length of the coil flyback waveform, so that at the end of that waveform the mosfet turns on agan, avoiding heat generation in the mosfet body diode and/or TVS diode.
* Mosfet ON time for subsequent cycles in the burst. Long enough for the peak mosfet voltage to just reach the target.