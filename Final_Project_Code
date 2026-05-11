#define RDA 0x80
#define TBE 0x20
#define SERVO1_PIN 9
#define SERVO2_PIN 10
#define DHT11_PIN 8
#define INTERRUPT_PIN 2
#include <Servo.h>
#include <DHT.h>
//#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal.h>

// --- Register Pointers ---
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;

volatile unsigned char *my_ADMUX = (unsigned char *)0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char *)0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int *)0x78;

volatile unsigned char *myTCCR1A = (unsigned char *)0x80;
volatile unsigned char *myTCCR1B = (unsigned char *)0x81;
volatile unsigned char *myTIMSK1 = (unsigned char *)0x6F;
volatile unsigned char *myTIFR1 = (unsigned char *)0x36;
volatile unsigned int *myTCNT1 = (unsigned int *)0x84;

volatile unsigned char *my_PORTA = (unsigned char *)0x22;
volatile unsigned char *my_DDRA = (unsigned char *)0x21;
volatile unsigned char *my_PINA = (unsigned char *)0x20;

volatile unsigned char *my_PORTB = (unsigned char *)0x25;
volatile unsigned char *my_DDRB = (unsigned char *)0x24;
volatile unsigned char *my_PINB = (unsigned char *)0x23;

volatile unsigned char *my_PORTC = (unsigned char *)0x28;
volatile unsigned char *my_DDRC = (unsigned char *)0x27;
volatile unsigned char *my_PINC = (unsigned char *)0x26;

volatile unsigned char *my_PORTD = (unsigned char *)0x2B;
volatile unsigned char *my_DDRD = (unsigned char *)0x2A;
volatile unsigned char *my_PIND = (unsigned char *)0x29;

volatile unsigned char *my_PORTE = (unsigned char *)0x2E;
volatile unsigned char *my_DDRE = (unsigned char *)0x2D;
volatile unsigned char *my_PINE = (unsigned char *)0x2C;

volatile unsigned char *my_PING  = (unsigned char *)0x32;
volatile unsigned char *my_DDRG  = (unsigned char *)0x33;
volatile unsigned char *my_PORTG = (unsigned char *)0x34;

volatile unsigned char *my_PINH  = (unsigned char *)0x100;
volatile unsigned char *my_DDRH  = (unsigned char *)0x101;
volatile unsigned char *my_PORTH = (unsigned char *)0x102;


// --- Global Variables ---
volatile unsigned int timer_seconds = 0;
volatile unsigned int desired_level = 0;
volatile unsigned int desired_temp = 0;
volatile unsigned int overflow_count = 0;
volatile unsigned char state = 0;
volatile bool monitoring_active = false;
volatile bool sampleFlag = false;
volatile bool start = false;
unsigned long lastButtonTime = 0;
// servo
volatile unsigned int servoPos = 0;
Servo servo1;
Servo servo2;
// dht
DHT dht11(DHT11_PIN, DHT11);
// rtc
RTC_DS3231 rtc;
// lcd
const int rs = 53, en = 52, d4 = 16, d5 = 17, d6 = 18, d7 = 19;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// --- Forward Declarations ---
void U0Init(int baud);
void putChar(unsigned char c);
void uart_print(const char *str);
void printNumber(unsigned int val);
unsigned int my_atoi(const char *s);
void m_delay(unsigned long ms);
void adc_init();
unsigned int adc_read(unsigned char channel);
void printCurrentTime();
void update_7seg(unsigned char val);
void startISR();

void setup() {
  *my_DDRD |= 0xF0;
  *my_DDRB |= 0x03;
  *my_DDRA |= 0xFC;
  *my_DDRC |= 0x81;
  //buttons
  *my_DDRE &= ~(1 << 4);
  *my_PORTE |= (1 << 4);
  *my_DDRA &= 0xFC;
  *my_PORTA |= 0x03;
  //LEDs
  *my_DDRE |= 0x28;
  *my_DDRG |= 0x20;
  *my_DDRH |= 0x18;

  U0Init(9600);
  adc_init();
  dht11.begin();
  //Wire.begin();
  rtc.begin();
  if (!rtc.begin()) {
    uart_print("\n\rRTC NOT FOUND!\n\r");
    while (1)
      ;
  }

  // --- THE BOX FIX ---
  m_delay(200);  // Wait for board power to stabilize

  // Clear any hardware buffer noise
  volatile unsigned char dummy;
  while (*myUCSR0A & RDA) dummy = *myUDR0;

  // ANSI Escape sequence to Clear Screen and Home Cursor
  // This wipes away the boxes in most Serial Monitors
  putChar(0x1B);
  uart_print("[2J");
  putChar(0x1B);
  uart_print("[H");

  // LCD Init
  lcd.begin(16, 2);
  lcd.clear();

  // Servo Init
  servo1.write(0);  // set both to closed
  servo2.write(0);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  // RTC init, uncomment to calibrate time
  //rtc.adjust(DateTime(2026, 5, 9, 6, 02, 0));

  // Timer Setup
  *myTCCR1A = 0x00;
  *myTCCR1B = 0x00;
  *myTCNT1 = 3036;
  *myTIFR1 |= 0x01;
  *myTIMSK1 |= 0x01;
  *myTCCR1B = 0x04;


  // Interrupt Setup
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), startISR, FALLING);

  //LED setup
  *my_PORTH |= 0x08;
  *my_PORTE &= ~0x28;
  *my_PORTG &= ~0x20;
  *my_PORTH &= ~0x10;
  

  // Formatted Header
  uart_print("\n\n\n");
  uart_print("==============================\n\r");
  uart_print("System Initialized & Ready\n\r");
  uart_print("System OFF. Press button to start.\n\r");
  uart_print("==============================\n\r");
  update_7seg(state);
}

void loop() {
  static char buffer[16];
  static int idx = 0;
  static int config_step = -1;

  // ON button, start CONFIG
  if (start) {
    start = false;
    if (millis() - lastButtonTime > 200) {  //debounce
      lastButtonTime = millis();
      if (state == 0) {
        monitoring_active = false;
        state = 1;
        *my_PORTH &= ~0x08;
        *my_PORTE &= ~0x28;
        *my_PORTG &= ~0x20;
        *my_PORTH |= 0x10;
        servo1.write(0);
        servo2.write(0);
        uart_print("\n\r");
        printCurrentTime();
        uart_print(" - ON\n\r");
        update_7seg(state);
        
        uart_print("Desired Sampling Increment (s): ");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Initializing.");
        m_delay(250);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Initializing..");
        m_delay(250);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Initializing...");
        m_delay(250);
        lcd.clear();

        config_step = 0;
        idx = 0;
        return;
      }
    }
  }

  // OFF button
  if (!(*my_PINA & (1 << 0))) {
    if (state != 0) {
      m_delay(50);
      state = 0;
      monitoring_active = false;
      *my_PORTH |= 0x08;
      *my_PORTE &= ~0x28;
      *my_PORTG &= ~0x20;
      *my_PORTH &= ~0x10;
      servo1.write(0);
      servo2.write(0);
      config_step = -1;
      uart_print("\n\n\n");
      uart_print("==============================\n\r");
      printCurrentTime();
      uart_print(" - System Shutdown. OFF.\n\r");
      uart_print("==============================\n\r");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SYSTEM OFF");
      update_7seg(state);
      return;
      }
  }

  // RESET button
  if (!(*my_PINA & (1 << 1))) {
    if (state == 4 || state == 3) {
      m_delay(50);
      printCurrentTime();
      uart_print(" - System RESET");
      state = 2;
      *my_PORTE |= 0x20;
      *my_PORTH &= ~0x18;
      *my_PORTE &= ~0x08;
      *my_PORTG &= ~0x20;
      monitoring_active = true;
    }
  }
  
  // CONFIGURATION
  if (*myUCSR0A & RDA) {
    unsigned char in = *myUDR0;

    putChar(in);

    if (in == '\n' || in == '\r') {
      buffer[idx] = '\0';
      if (idx > 0) {
        if (idx == 1 && buffer[0] == '0') {
          state = 0;
          *my_PORTH |= 0x08;
          *my_PORTE &= ~0x28;
          *my_PORTG &= ~0x20;
          *my_PORTH &= ~0x10;
          servo1.write(0);
          servo2.write(0);
          monitoring_active = false;
          config_step = -1;
          idx = 0;
          uart_print("\n\rInvalid entry. Reset to OFF.\n\r");
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("SYSTEM OFF");
          update_7seg(state);
          return;
        }

        if (config_step == 0) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Configuring...");
          timer_seconds = my_atoi(buffer);
          uart_print("\n\rDesired Level Control: ");
          config_step = 1;
        } else if (config_step == 1) {
          desired_level = my_atoi(buffer);
          uart_print("\n\rDesired Temperature Control: ");
          config_step = 2;
        } else if (config_step == 2) {
          desired_temp = my_atoi(buffer);
          uart_print("\n\rMonitoring Started\n\r");
          monitoring_active = true;
          config_step = 3;
        }
      }
      idx = 0;
    } else if (idx < 15) {
      if (in >= '0' && in <= '9') {
        buffer[idx++] = in;
      }
    }
  }
  
  // IDLE, ACTIVE, and ERROR
  if (sampleFlag) {

    unsigned int sensorVal = adc_read(0);
    int temp = dht11.readTemperature();

    if (isnan(temp)) {
      uart_print("\n\rDHT11 Read Failed!\n\r");
      state = 4;
      *my_PORTE &= ~0x28;
      *my_PORTG |= 0x20;
      *my_PORTH &= ~0x18;
      return;
    }

    servoPos = map(sensorVal, 0, 1023, 0, 180);

    if (sensorVal == 1023 || sensorVal == 0 || temp == 0 || isnan(sensorVal) || isnan(temp)) {
      state = 4;
      *my_PORTE &= ~0x28;
      *my_PORTG |= 0x20;
      *my_PORTH &= ~0x18;
      monitoring_active = false;
      servo1.write(0);
      servo2.write(0);
      uart_print("\n\n");
      printCurrentTime();
      uart_print(" - ERROR: sensor fault");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("ERROR");
      lcd.setCursor(0,1);
      lcd.print("Please Reset");
      update_7seg(state);
    } 
    else if (sensorVal > desired_level || temp > desired_temp) {
      state = 3;
      *my_PORTE |= 0x08;
      *my_PORTE &= ~0x20;
      *my_PORTG &= ~0x20;
      *my_PORTH &= ~0x18;
      if (temp > desired_temp) {
        servo2.write(180);
      }
      servo1.write(servoPos);
      update_7seg(state);

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Water lvl:");
      lcd.setCursor(11,0);
      lcd.print(sensorVal);

      lcd.setCursor(0,1);
      lcd.print("Temperature:   C");
      lcd.setCursor(13,1);
      lcd.print(temp);
    } 
    else {
      state = 2;
      *my_PORTE |= 0x20;
      *my_PORTH &= ~0x18;
      *my_PORTE &= ~0x08;
      *my_PORTG &= ~0x20;
      servo1.write(0);
      servo2.write(0);
      update_7seg(state);

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Water lvl:");
      lcd.setCursor(11,0);
      lcd.print(sensorVal);

      lcd.setCursor(0,1);
      lcd.print("Temperature:   C");
      lcd.setCursor(13,1);
      lcd.print(temp);
    }
    uart_print("\n\r---------------------------");
    uart_print("\n\rINTERVAL TRIGGERED: ");
    printCurrentTime();
    uart_print("\n\r > ADC SENSOR READING: ");
    printNumber(sensorVal);
    uart_print("\n\r > TEMPERATURE READING: ");
    printNumber(temp);
    uart_print(" C");
    uart_print("\n\r > TIMER INCREMENT: ");
    printNumber(timer_seconds);
    uart_print(" s");
    uart_print("\n\r > WATER THRESHOLD: ");
    printNumber(desired_level);
    uart_print("\n\r > MAX TEMP THRESHOLD: ");
    printNumber(desired_temp);    
    uart_print("\n\r > SYSTEM STATE: ");
    printNumber(state);
    uart_print("\n\r---------------------------\n\r");
  }
  sampleFlag = false;
}

void U0Init(int baud) {
  unsigned int tbaud = (16000000 / 16 / baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}

void putChar(unsigned char c) {
  while (!(*myUCSR0A & TBE))
    ;
  *myUDR0 = c;
}

void uart_print(const char *str) {
  while (*str) putChar(*str++);
}

void printNumber(unsigned int val) {
  if (val == 0) {
    putChar('0');
    return;
  }
  unsigned char digits[5];
  int i = 0;
  while (val > 0) {
    digits[i++] = (val % 10) + '0';
    val /= 10;
  }
  for (int j = i - 1; j >= 0; j--) putChar(digits[j]);
}

void print2Digits(unsigned int val) {
  if (val < 10) putChar('0');

  printNumber(val);
}

unsigned int my_atoi(const char *s) {
  unsigned int res = 0;
  while (*s >= '0' && *s <= '9') {
    res = res * 10 + (*s - '0');
    s++;
  }
  return res;
}

void m_delay(unsigned long ms) {
  for (volatile unsigned long i = 0; i < ms * 1500; i++)
    ;
}

void adc_init() {
  *my_ADCSRA |= 0x80;
  *my_ADCSRA &= ~0x20;
  *my_ADCSRA &= ~0x08;
  *my_ADCSRA &= ~0x07;
  *my_ADCSRB &= ~0x0F;
  *my_ADMUX = 0x40;
}

unsigned int adc_read(unsigned char channel) {
  *my_ADMUX = (*my_ADMUX & 0xE0) | (channel & 0x1F);
  *my_ADCSRA |= 0x40;
  while ((*my_ADCSRA & 0x40) != 0)
    ;
  return *my_ADC_DATA;
}

void printCurrentTime() {
  DateTime now = rtc.now();
  uart_print("(");
  print2Digits(now.hour());
  uart_print(":");
  print2Digits(now.minute());
  uart_print(":");
  print2Digits(now.second());
  uart_print(")");
}

void update_7seg(unsigned char val) {
  *my_PORTA &= ~0xFC;
  *my_PORTC &= ~0x81;

  switch (val) {
    case 0:  // a b c d e f
      *my_PORTA |= 0xFC;
      break;
    case 1:  // b c
      *my_PORTA |= 0x18;
      break;
    case 2:  // a b d e g
      *my_PORTA |= 0x6C;
      *my_PORTC |= 0x80;
      break;
    case 3:  // a b c d g
      *my_PORTA |= 0x3C;
      *my_PORTC |= 0x80;
      break;
    case 4:  // b c f g
      *my_PORTA |= 0x98;
      *my_PORTC |= 0x80;
      break;
  }
}

ISR(TIMER1_OVF_vect) {
  *myTCCR1B &= 0xF8;
  *myTCNT1 = 3036;
  *myTCCR1B |= 0x04;

  if (state != 0 && monitoring_active) {
    overflow_count++;

    if (overflow_count >= timer_seconds) {
      overflow_count = 0;
      sampleFlag = true;
    }
  }
}

void startISR() {
  start = true;
}
