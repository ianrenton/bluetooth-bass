# Billy Bass Sat Nav / Bluetooth Bass

A project by Ian Renton. For more details, see https://ianrenton.com/billy-bass-sat-nav

## Toolchain

This project uses the Platform.io environment, and is designed to run on a DOIT ESP32 Devkit (or clone), using the Arduino toolkit.

## Operation

The software is designed to work with hardware as described at the link above.

When started, the ESP32 provides a Bluetooth A2DP profile target called "Billy Bass". Connect to this using your phone or other source of Bluetooth audio.

Received audio is piped out over I2S to a suitable receiver and audio amplifier, to be played on the Billy Bass speaker.

Audio within a certain configurable frequency range is detected, and used to trigger Billy Bass head and mouth motion.

At the moment this is basically "any audio", so the effect is quite good for things like Sat Nav instructions, Google Assistant/Siri responses, etc. It's not great for general music (see "Future Work" section below).

# Notes

This is my first time doing multi core processing on ESP32, go easy on me :)

# Future Work

What I really want to do long term with this project is to have on-the-fly detection of vocals from music, so that the head and mouth only move to vocals, and for non-vocal sections of music a beat detector allows the tail to flap in sync to the music. However, this is likely beyond the capability of the ESP32&mdash;the flash on my device is 90% full just with the current I2S stream & FFT code.
