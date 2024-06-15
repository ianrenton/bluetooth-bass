// Billy Bass Sat Nav / "BLE Bass" control code
// by Ian Renton, 2024. CC Zero / Public Domain
// Based on examples from https://github.com/tierneytim/btAudio & https://github.com/kosme/arduinoFFT
// Libraries used with many thanks.

// Bluetooth device name
#define DEVICE_NAME "Billy Bass"

// Timing
#define MOUTH_FLAP_INTERVAL_MILLIS 100 // Rate of mouth movement. Lower number = faster
#define SERIAL_BAUD 115200

// FFT parameters. Hopefully these match the sampling frequency and number of samples per packet
// coming from your device. This is what works with my phone!
#define SAMPLING_FREQUENCY 44100
#define SAMPLES_PER_FFT 2048

// I2S pins
#define I2S_BCLK_PIN 18
#define I2S_WS_PIN 19
#define I2S_DOUT_PIN 21

// Motor control pins
#define HEADTAIL_MOTOR_PIN_1 12
#define HEADTAIL_MOTOR_PIN_2 14
#define HEADTAIL_MOTOR_PWM_PIN 13
#define MOUTH_MOTOR_PIN_1 27
#define MOUTH_MOTOR_PIN_2 26
#define MOUTH_MOTOR_PWM_PIN 25

// Misc pins
#define LED_PIN 2
#define BUTTON_PIN 4

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

// Data storage for FFT, FFT object & task
double vReal[SAMPLES_PER_FFT];
double vImag[SAMPLES_PER_FFT];
ArduinoFFT<double> fft = ArduinoFFT<double>(vReal, vImag, SAMPLES_PER_FFT, SAMPLING_FREQUENCY);

// Data storage & mutex for passing audio samples between CPUs
double audioTransferBuffer[SAMPLES_PER_FFT];
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

// Create Bluetooth autio receiver
btAudio audio = btAudio(DEVICE_NAME);

// Setup and run the program
void setup()
{
  // Set up serial
  Serial.begin(SERIAL_BAUD);

  // Set up debug LED
  pinMode(LED_PIN, OUTPUT);

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
  audio.I2S(I2S_BCLK_PIN, I2S_DOUT_PIN, I2S_WS_PIN);

  // Set our custom sink callback; this will pass audio data to I2S but also store the
  // data for FFT
  audio.setSinkCallback(sinkCallback);
}

// Main program loop. Runs on core 1, calculates the FFT of the latest audio sample and
// moves the fish motors accordingly.
void loop()
{
  // Copy data out of the inter-process audio transfer buffer into the "real" data buffer
  // that will be used for the FFT. This is controlled with a mutex lock to ensure we
  // don't read from it and write to it at the same time.
  xSemaphoreTake(mutex, portMAX_DELAY);
  for (int i = 0; i < SAMPLES_PER_FFT; i++)
  {
    vReal[i] = audioTransferBuffer[i];
  }
  xSemaphoreGive(mutex);
  // Zero out the imaginary data buffer, leaving only the real data buffer written to
  // by the bluetooth-to-I2S callback
  for (int i = 0; i < SAMPLES_PER_FFT; i++)
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
  // if not, then rest both motors.
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
  int samplesReceived = len / 2;
  // Handle input as 16-bit data
  int16_t* data16 = (int16_t *)data;

  // Write to I2S
  size_t i2s_bytes_write = 0;
  for (int i = 0; i < samplesReceived; i++)
  {
    i2s_write(I2S_NUM_0, data16, 2, &i2s_bytes_write, 10);
    data16++;
  }

  // Reset data pointer
  data16 = (int16_t *)data;

  // Write to the audio transfer buffer to pass this data over to the FFT task on
  // the other CPU. This is controlled with a mutex lock to ensure we don't read
  // from it and write to it at the same time.
  xSemaphoreTake(mutex, portMAX_DELAY);
  for (int i = 0; i < min(samplesReceived, SAMPLES_PER_FFT); i++)
  {
    audioTransferBuffer[i] = *data16;
    data16++;
  }
  xSemaphoreGive(mutex);
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
  digitalWrite(LED_PIN, HIGH); // Debug
}

// Close the fish's mouth
void mouthClose()
{
  digitalWrite(MOUTH_MOTOR_PIN_1, HIGH);
  digitalWrite(MOUTH_MOTOR_PIN_2, LOW);
  digitalWrite(LED_PIN, LOW); // Debug
}

// Rest the fish's mouth
void mouthRest()
{
  digitalWrite(MOUTH_MOTOR_PIN_1, LOW);
  digitalWrite(MOUTH_MOTOR_PIN_2, LOW);
  digitalWrite(LED_PIN, LOW); // Debug
}