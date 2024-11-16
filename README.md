# Ultranet-to-I2S
Use RPi Pico to decode Behringer Ultranet and output to multiple I2S streams

I've wanted a simple (and cheap!) way to decode analog audio from Behringer Ultranet for years, and I've finally got around to doing it! The Raspberry Pi Pico is the ideal device to use, as it has two secret weapons: the PIO modules, and dual processor. I use the PIO modules to decode an incoming Ultranet stream into 8 audio channels, then to encode pairs of channels into I2S output streams (to send to low-cost I2S decoder boards as found on popular auction sites). 

Dual Processors provide a very simple way to resolve the synchronisation issue between the incoming Ultranet stream (which is clocked from the source device) and the outgoing I2S streams, which are clocked from this device. The two clocks can be made similar, but not identical. Using one processor to read the Ultranet audio values into an array, then another processor to output them to I2S means that each thread can run independently, thus eliminating the clicks and noise associated with out-of-sync digital audio.

The third feature of the RPi Pico I found very useful is it's ability to over-clock. Running the system clock at 172MHz and dividing by 7 into the PIO modules gives an operatingi frequency very close to the digital audio streaming speed of 12.288MHz (48KHz sampling, 8 channels). I've run several "cheap" chinese pico knock-off boards at this frequency with no issues whatsoever (they all use the same RPi RP2040 chip, so this shouldn't be a surprise).

The project uses the Raspberry Pi Foundation Pico C/C++ SDK and Visual Studio Code as the development environment.

I've borrowed PIO program ideas from two excellent github projects:

https://github.com/elehobica/pico_spdif_rx - I modified the PIO program from this project, as the Ultranet decode (very similar to AES/EBU)

https://github.com/nyh-workshop/rpi-pico-i2sExample - I used the I2S output encoder PIO program from this project

Thanks a lot to the authors of the two excellent projects above!

