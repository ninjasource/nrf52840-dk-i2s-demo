# I2S demo for the nRF52840-DK

A demo of the nrf52840-dk playing music through a UDA1334A DAC using I2S. 
This demo uses the `lc3-codec` to decode LC3 encoded audio into PCM (wave) encoded audio to be fed directly to the DAC. 
In order to keep the demo simple, the audio file is embedded directly into the flash of the mcu.

![Example Setup](https://github.com/ninjasource/nrf52840-dk-i2s-demo/blob/main/nrf52840-i2s-demo.jpeg?raw=true)

# Requirements

## Hardware

- `nRF52480-DK` development kit
- `UDA1334A I2S DAC` 
- Pins (DAC -> dk): 
  - 3.3v -> 3.3v
  - GND -> GND
  - WSEL -> P0_31
  - DIN -> P0_30
  - BCLK -> P0_29
  
## Software

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-run
```

# Running

```
cargo run --release
```

# How it works

The application uses a runtime called `RTIC` to help with the real-time nature of audio processing. 

The `init` function is called once on startup and sets up the I2S peripheral as well as initializing the queue and buffers used to shuffle audio data around.

The `idle` loop reads the LC3 encoded audio file over and over again, decoding each 10ms audio frame and pushing it onto a queue to be processed by another task.
If the queue is full the idle loop will block until there is another vacant position. 
It is important that the audio processing that gets done in the idle loop never takes longer than the length of an audio frame (10ms). If it does you will notice jumpiness in the audio. 
There is a `stats` module that measures how long the decoding process takes and prints it to the console via defmt and probe-run.

The `i2s_transfer` function is called every time the MCU fires the I2S interrupt. 
This typically happens after the DMA transfer to the destination peripheral (the UDA1334A DAC) is complete.
We have a double buffer setup where we write to one buffer as the DMA transfer is happening on the other buffer. 
We dequeue data (originating from our `idle` loop) and copy it to the tx `buffer` we intend to send.
We toggle between the two buffers and set up the next transfer using the `i2s.tx(buffer)` function call. 
This i2s transfer interrupt fires regardless of whether or not the `idle` loop has generated enough data for it so if there is no data we fill the space with silence (zero the buffer). This should not happen unless decoding is too slow to keep up.

# Troubleshooting

If you hear noise on the line then you may have a dodgy connection somewhere.