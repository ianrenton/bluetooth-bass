// Billy Bass Sat Nav / "BLE Bass" control code
// by Ian Renton, 2024. CC Zero / Public Domain
// Based on examples from https://github.com/tierneytim/btAudio & https://github.com/kosme/arduinoFFT
// Libraries used with many thanks.

// Bluetooth device name
#define DEVICE_NAME "Billy Bass"

// Timing
#define MOUTH_FLAP_INTERVAL_MILLIS 100 // Rate of mouth movement. Lower number = faster

// Motor control pins
#define HEADTAIL_MOTOR_PIN_1 12
#define HEADTAIL_MOTOR_PIN_2 14
#define HEADTAIL_MOTOR_PWM_PIN 13
#define MOUTH_MOTOR_PIN_1 27
#define MOUTH_MOTOR_PIN_2 26
#define MOUTH_MOTOR_PWM_PIN 25

// Motor PWM settings
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8
#define HEADTAIL_MOTOR_PWM_CHANNEL 0
#define MOUTH_MOTOR_PWM_CHANNEL 1
#define HEADTAIL_MOTOR_PWM_DUTY_CYCLE 255 // Proxy for motor speed, up to 2^resolution
#define MOUTH_MOTOR_PWM_DUTY_CYCLE 255    // Proxy for motor speed, up to 2^resolution

// Includes
#include <Arduino.h>
#include <btAudio.h>
#include <arduinoFFT.h>

// Function defs
void setup();
void loop();
void sinkCallback(const uint8_t *data, uint32_t len);
void calcFFT(void *pvParameters);
void headOut();
void headTailRest();
void flapMouth();
void mouthOpen();
void mouthClose();
void mouthRest();

// FFT parameters. Hopefully these match the sampling frequency and number of samples per packet
// coming from your device. This is what works on my phone!
const double samplingFrequency = 44100;
const int numSamples = 2048;

// Data storage for FFT, FFT object & task
double vReal[numSamples];
double vImag[numSamples];
ArduinoFFT<double> fft = ArduinoFFT<double>(vReal, vImag, numSamples, samplingFrequency);

// Create Bluetooth autio receiver
btAudio audio = btAudio(DEVICE_NAME);

// For inter-process communication, track whether we are receiving audio currently
boolean receivingAudio = false;

// Setup and run the program
void setup()
{
  // Set up serial
  Serial.begin(115200);

  // Set up motor control pins
  pinMode(HEADTAIL_MOTOR_PIN_1, OUTPUT);
  pinMode(HEADTAIL_MOTOR_PIN_2, OUTPUT);
  pinMode(HEADTAIL_MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOUTH_MOTOR_PIN_1, OUTPUT);
  pinMode(MOUTH_MOTOR_PIN_2, OUTPUT);
  pinMode(MOUTH_MOTOR_PWM_PIN, OUTPUT);

  // Set up PWM
  ledcSetup(HEADTAIL_MOTOR_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(MOUTH_MOTOR_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(HEADTAIL_MOTOR_PWM_PIN, HEADTAIL_MOTOR_PWM_CHANNEL);
  ledcAttachPin(MOUTH_MOTOR_PWM_PIN, MOUTH_MOTOR_PWM_CHANNEL);
  ledcWrite(HEADTAIL_MOTOR_PWM_CHANNEL, HEADTAIL_MOTOR_PWM_DUTY_CYCLE);
  ledcWrite(MOUTH_MOTOR_PWM_CHANNEL, MOUTH_MOTOR_PWM_DUTY_CYCLE);

  // Advertise Bluetooth device
  audio.begin();

  // Try to automatically reconnect to previously connected Bluetooth device if possible
  audio.reconnect();

  // Set up streaming audio over I2S
  int bck = 26;
  int ws = 27;
  int dout = 25;
  audio.I2S(bck, dout, ws);

  // Set our custom sink callback; this will pass audio data to I2S but also store the
  // data for FFT
  audio.setSinkCallback(sinkCallback);
}

// Main program loop. Runs on core 1, calculates the FFT of the latest audio sample and
// moves the fish motors accordingly.
void loop()
{
  // Zero out the imaginary data buffer, leaving only the real data buffer written to
  // by the bluetooth-to-I2S callback
  for (int i = 0; i < numSamples; i++)
  {
    vImag[i] = 0.0;
  }
  // Compute the FFT
  fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude();
  // Do we have some power in the FFT?
  boolean receivingAudio = !isnan(fft.majorPeak());

  // Work out if we are receiving audio. If so, stick the fish head out and flap the mouth,
  // if not bring it in.
  Serial.println(receivingAudio); // @todo remove
  if (receivingAudio)
  {
    headOut();
    flapMouth();
  }
  else
  {
    headTailRest();
    mouthRest();
  }

  // Short delay to give FreeRTOS some breathing space to not trigger the watchdog
  delay(10);
}

// Callback for processing an audio data buffer arrived from Bluetooth. Sends the
// data to I2S but also populates a second buffer for computing the FFT.
void sinkCallback(const uint8_t *data, uint32_t len)
{
  int numSamples = len / 2;
  // Handle input as 16-bit data
  int16_t *data16 = (int16_t *)data;

  size_t i2s_bytes_write = 0;
  for (int i = 0; i < numSamples; i++)
  {
    // Write to I2S
    i2s_write(I2S_NUM_0, data16, 2, &i2s_bytes_write, 10);

    // Write to FFT data buffer
    vReal[i] = *data16;

    data16++;
  }
}

// Bring the fish's head out
void headOut()
{
  digitalWrite(HEADTAIL_MOTOR_PIN_1, LOW);
  digitalWrite(HEADTAIL_MOTOR_PIN_2, HIGH);
}

// Put the fish head and tail back to the neutral position
void headTailRest()
{
  digitalWrite(HEADTAIL_MOTOR_PIN_1, LOW);
  digitalWrite(HEADTAIL_MOTOR_PIN_2, LOW);
}

// Flap the fish's mouth once at the required rate
void flapMouth()
{
  mouthOpen();
  delay(MOUTH_FLAP_INTERVAL_MILLIS);
  mouthClose();
  delay(MOUTH_FLAP_INTERVAL_MILLIS);
}

// Open the fish's mouth
void mouthOpen()
{
  digitalWrite(MOUTH_MOTOR_PIN_1, LOW);
  digitalWrite(MOUTH_MOTOR_PIN_2, HIGH);
}

// Close the fish's mouth
void mouthClose()
{
  digitalWrite(MOUTH_MOTOR_PIN_1, HIGH);
  digitalWrite(MOUTH_MOTOR_PIN_2, LOW);
}

// Rest the fish's mouth
void mouthRest()
{
  digitalWrite(MOUTH_MOTOR_PIN_1, LOW);
  digitalWrite(MOUTH_MOTOR_PIN_2, LOW);
}