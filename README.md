# Ultranet-to-I2S
Use RPi Pico to decode Behringer Ultranet and output to multiple I2S streams

I've wanted a simple (and cheap!) way to decode analog audio from Behringer Ultranet for years, and I've finally got around to doing it! The Raspberry Pi Pico is the ideal device to use, as it has two secret weapons: the PIO modules, and dual processors. I use the PIO modules to decode an incoming Ultranet stream into 8 audio channels, then to encode pairs of channels into I2S output streams (to send to low-cost I2S decoder boards as found on popular auction sites). 

Dual Processors provide a very simple way to resolve the synchronisation issue between the incoming Ultranet stream (which is clocked from the source device) and the outgoing I2S streams, which are clocked from this device. The two clocks can be made similar, but not identical. Using one processor to read the Ultranet audio values into an array, then another processor to output them to I2S means that each thread can run independently, thus eliminating the clicks and noise associated with out-of-sync digital audio.

The third feature of the RPi Pico I found very useful is it's ability to over-clock. Running the system clock at 172MHz and dividing by 7 into the PIO modules gives an operating frequency very close to the digital audio streaming speed of 12.288MHz (48KHz sampling, 8 channels). I've run several "cheap" chinese pico knock-off boards at this frequency with no issues whatsoever (they all use the same RPi RP2040 chip, so this shouldn't be a surprise).

The project uses the Raspberry Pi Foundation Pico C/C++ SDK (V2.0.0) and Visual Studio Code as the development environment (with the RPi Pico Extension plugin).

I've borrowed PIO program ideas from two excellent github projects:

https://github.com/elehobica/pico_spdif_rx - I modified the PIO program from this project, as the Ultranet protocol is very similar to AES/EBU

https://github.com/nyh-workshop/rpi-pico-i2sExample - I used the I2S output encoder PIO program from this project

Thanks a lot to the authors of the two excellent projects above!

Here is a pic of my prototype board. It's based on a PCB I made for Ultranet fan-out (one in, four out) as a "mini P16-D". I just use the input buffer chip (an MC3486) and the 5V power regulator. The rest of the board space I used to attach the pico (chinese mini knock-off) and the output decoder boards(PCM5102A modules from your favourite auction site).
![PicoUltranetDecoder](https://github.com/user-attachments/assets/e9e0c4d6-15a5-412d-a558-9ded5bb5acc3)

In order to receive the Ultranet streams into the Pico, a line receiver IC needs to be used, to convert the differential signals from the Ultranet cable into single-ended TTL compatible pulses for the Pico. The following diagram shows a circuit I use for this purpose:

[Schematic_UltranetInput_2025-01-01-2.pdf](https://github.com/user-attachments/files/18286932/Schematic_UltranetInput_2025-01-01-2.pdf)

To Do:
  In the next few weeks I plan to test with current version of pico-sdk (2.1.1 at time of writing) and pico 2 (RP2350) now that I've actually purchased one!!


Feedback:
  Please let me know via comments if you have any issues with overclocking your pico - the code by default overclocks to a higher level than strictly necessary, and can be reduced easily if anyone is getting issues (I never have, with a dozen or so cheap chinese pico knock-offs!!). The current code overclocks to 172MHz, then divides by 7 in the Ultranet PIO input module. This gives the most accurate tracking of Ultranet signal edges as our sample period is 7 times shorter than the Ultranet bit period. I've tested at lower clock speeds (and dividing by corrispondingly lower amounts) but 172MHz seems very stable for every pico I've tried.
  
