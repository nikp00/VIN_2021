#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <I2Cdev.h>
#include <Simple_MPU6050.h>

// OLED screen
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define OLED_RESET 4        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 400000U, 400000U);

// MPU6050
Simple_MPU6050 mpu;
ENABLE_MPU_OVERFLOW_PROTECTION();
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) // (BLACK BOX) Ya, don't complain that I used "for(;;){}" instead of "if(){}" for my Blink Without Delay Timer macro. It works nicely!!!
#define OFFSETS 172, 277, 2283, 72, -33, 44
#define MPU6050_ADDRESS_AD0_LOW 0x68  // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH 0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS MPU6050_ADDRESS_AD0_LOW
Quaternion q;
VectorFloat gravity;
float ypr[3];
float xyz[3];

// CPPM
#define CHANNEL_NUMBER 12          //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500 //set the default servo value
#define CHANNEL_MIN_VALUE 1000     //set the min servo value
#define CHANNEL_MAX_VALUE 2000     //set the max servo value
#define FRAME_LENGTH 22500         //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300           //set the pulse length
#define CPPM_MODULATION 1          //set polarity of the pulses: 1 is positive, 0 is negative
#define CPPM_PIN 10                //set PPM signal output pin on the arduino
#define ROLL_CHANNEL 0
#define THROTTLE_CHANNEL 1
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3

int ppm[CHANNEL_NUMBER];

// Flex sensor
#define FLEX_SENSOR_PIN 21 // A7
#define FLEX_MAX_VALUE 800
#define FLEX_MIN_VALUE 300
#define N_SAMPLES 10

int flex_readings[N_SAMPLES];
uint8_t flex_reading_index = 0;

// Debuging LED
#define LED_PIN 7
bool ledState = false;

ISR(TIMER1_COMPA_vect)
{
    static boolean state = true;

    TCNT1 = 0;

    if (state)
    {
        digitalWrite(CPPM_PIN, CPPM_MODULATION);
        OCR1A = PULSE_LENGTH * 2;
        state = false;
    }
    else
    {
        static byte cur_chan_numb;
        static unsigned int calc_rest;

        digitalWrite(CPPM_PIN, !CPPM_MODULATION);
        state = true;

        if (cur_chan_numb >= CHANNEL_NUMBER)
        {
            cur_chan_numb = 0;
            calc_rest = calc_rest + PULSE_LENGTH; //
            OCR1A = (FRAME_LENGTH - calc_rest) * 2;
            calc_rest = 0;
        }
        else
        {
            OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
            calc_rest = calc_rest + ppm[cur_chan_numb];
            cur_chan_numb++;
        }
    }
}

void drawRollPitchYawThrottle(int roll, int pitch, int yaw, int throttle)
{
    roll = map(roll, 2000, 1000, 0, 59);
    pitch = map(pitch, 1000, 2000, 0, 32);
    yaw = map(yaw, 2000, 1000, 69, 128);
    throttle = map(throttle, 1000, 2000, 0, 32);
    display.clearDisplay();
    display.drawRoundRect(0, 0, 59, 32, 12, SSD1306_WHITE);
    display.drawRoundRect(69, 0, 59, 32, 12, SSD1306_WHITE);

    // Left stick
    display.drawPixel(yaw, throttle, SSD1306_WHITE);
    // Right stick
    display.drawPixel(roll, pitch, SSD1306_WHITE);
    display.display();
}

void initDisplay()
{
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println(F("SSD1306 allocation failed"));
    }

    display.display();
    delay(2000); // Pause for 2 seconds
    display.clearDisplay();
}

void readMPU(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp)
{
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);

    uint16_t yaw = (uint16_t)map(ypr[0] * 180 / M_PI, -90, 90, CHANNEL_MIN_VALUE, CHANNEL_MAX_VALUE);
    uint16_t pitch = (uint16_t)map(ypr[1] * 180 / M_PI, -90, 90, CHANNEL_MAX_VALUE, CHANNEL_MIN_VALUE);
    uint16_t roll = (uint16_t)map(ypr[2] * 180 / M_PI, -90, 90, CHANNEL_MIN_VALUE, CHANNEL_MAX_VALUE);

    ppm[ROLL_CHANNEL] = roll;
    ppm[YAW_CHANNEL] = yaw;
    ppm[PITCH_CHANNEL] = pitch;
}

void initMPU()
{
    Wire.begin();
    Wire.setClock(400000);
    mpu.SetAddress(MPU6050_DEFAULT_ADDRESS).load_DMP_Image(OFFSETS);
    mpu.on_FIFO(readMPU);
}

void initCPPM()
{
    //initiallize default ppm values
    for (int i = 0; i < CHANNEL_NUMBER; i++)
    {
        ppm[i] = CHANNEL_DEFAULT_VALUE;
    }

    pinMode(CPPM_PIN, OUTPUT);
    digitalWrite(CPPM_PIN, !CPPM_MODULATION); //set the PPM signal pin to the default state (off)

    cli();
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;

    OCR1A = 100;
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS11);   // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei();
}

void initFlexSensor()
{
    pinMode(FLEX_SENSOR_PIN, INPUT);
    analogRead(FLEX_SENSOR_PIN);
}

void readFlexSensor()
{
    float raw = analogRead(FLEX_SENSOR_PIN);
    flex_readings[flex_reading_index] = raw;
    flex_reading_index = (flex_reading_index + 1) % N_SAMPLES;

    Serial.println(raw);

    int avg = 0;
    for (int i = 0; i < N_SAMPLES; i++)
    {
        avg += flex_readings[i];
    }

    int throttle = map(avg / N_SAMPLES, FLEX_MIN_VALUE, FLEX_MAX_VALUE, CHANNEL_MAX_VALUE, CHANNEL_MIN_VALUE);
    ppm[THROTTLE_CHANNEL] = throttle;
}

void printChannels()
{
    Serial.print("YAW:\t");
    Serial.print(ppm[YAW_CHANNEL]);
    Serial.print("\t");
    Serial.print("PITCH:\t");
    Serial.print("\t");
    Serial.print(ppm[PITCH_CHANNEL]);
    Serial.print("\t");
    Serial.print("ROLL:\t");
    Serial.print(ppm[ROLL_CHANNEL]);
    Serial.print("\t");
    Serial.print("THROTTLE:\t");
    Serial.print(ppm[THROTTLE_CHANNEL]);
    Serial.print("\t");
    Serial.print("time::\t");
    Serial.println(millis());
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    Serial.begin(115200);
    initCPPM();
    initMPU();
    initFlexSensor();
    initDisplay();
}

void loop()
{
    mpu.dmp_read_fifo();

    readFlexSensor();

    digitalWrite(LED_PIN, ledState);
    ledState = !ledState;

    static long timer_display = millis();
    if ((long)(millis() - timer_display) >= 100)
    {
        timer_display = millis();
        drawRollPitchYawThrottle(ppm[ROLL_CHANNEL], ppm[PITCH_CHANNEL], ppm[YAW_CHANNEL], ppm[THROTTLE_CHANNEL]);
    }

    static long timer_serial = millis();
    if ((long)(millis() - timer_serial) >= 1000)
    {
        timer_serial = millis();
        printChannels();
    }
}
