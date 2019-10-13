# spectrum-analyzer
Spectrum analyzer for arduino

Project built using Arduino v1.8.5, FastLED v3.2.0, ArduinoFHT4


 * sampling speed is arduino clock speed/prescaler
 * resolution of each bin is sampling speed/# of bins
 * lower sampling speed + more bins = max resolution
 * max number of bins is 256, limiting resolution, so a higher prescaler should be used for better resolution to reduce sampling speed
 * using prescaler of 64 gives a frequency of 19,231 Hz
 * this means all frequences > ~9kHz must be filtered out to avoid aliasing (using LPF)
 * this will still capture the most apparent frequencies in music so all good, resolution of ~75 Hz/bin
 * run code should be kept to a minimum to improve processing times.  too much will break the FHT entirely
 
 Plugging something into the aux port will disconnect the mic input in hardware, no setting change required
 
 Additional modes/styles/etc can be added as desired, just fiddle with the map function
 
 Feel free to use and change as you wish, all I ask for is due credit :)
