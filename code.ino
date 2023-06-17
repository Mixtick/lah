#define F_CPU 128000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/sleep.h>
#include <EEPROM.h>
#include <util/delay.h>
#define MOTOR (1<<PB1)
#define SENSOR (1<<PB4)
#define UNGENUTZT (1<<PB0 | 1<<PB2 | 1<<PB3)
#define AUS 0
#define AN 1
#define WARTEND 2
#define ERSTMESSUNG 0
#define FOLGEMESSUNG 1
#define SCHALTDAUER 5
#define IMPULSANZAHL 3
#define EINGABEZEIT 3330
#define RUHEZEIT 6660
//#define MANIPULATION 2                       // 1 -> ab 11km/h   2 -> ab 22km/h
#define min_Tempo 111                        //  Einstellungen für Yamaha (Manipulation ab 11 km/h)

unsigned int  max_Tempo = EEPROM.get(40, max_Tempo);;
unsigned int  SetupModeAktiv = EEPROM.get(66, SetupModeAktiv);
unsigned long Radumfang = EEPROM.get(0, Radumfang);       // Radumfang / ( 1 / 3.6 ) * 2
uint16_t Kalibrierungswert = 333;
volatile uint16_t ausgabe = Radumfang / 5;
volatile uint16_t messung;
volatile uint16_t ueberlauf;
volatile uint8_t gemessen = 0;
volatile unsigned long guteminute = 0;
volatile unsigned long counterOut = 0;
volatile unsigned long counterDiff = EEPROM.get(80, counterDiff);
unsigned int SuperPursuitMode = EEPROM.get(166, SuperPursuitMode);
unsigned long SuperPursuitModeAlt = SuperPursuitMode;
volatile unsigned long counterIn = counterDiff;
volatile uint8_t messstatus = ERSTMESSUNG;
volatile uint8_t output = AUS;


void long_delay(uint16_t ms) {
  for (; ms > 0; ms--)
    _delay_ms(1);
}

inline uint8_t debounce(void) {
  static uint8_t state = 0;
  static uint8_t prevState = 0;
  static uint16_t openTime = 0;
  static uint16_t closedTime = 0;
  uint8_t result = 0;
  prevState = state;
  if (!(PINB & SENSOR)) {                       // Sensor is closed
    closedTime++;
    openTime = 0;
    state = 0;
  } else {                                      // Sensor is open
    openTime++;
    closedTime = 0;
    state = 1;
  }
  if (state != prevState) {                                     // Sensor state has changed
    if (state == 1 && openTime >= 19200 && result == 1) {       // Debounce open signal for ~150ms             (1 ms -> 128 cycles)
      result = 0;
      return result;
    }
    if (state == 0 && closedTime >= 2560 && result == 0) {      // Debounce closed signal for ~20ms, only if sensor result is open
      result = 1;
      return result;
    }
    return result;
  }
  return result;
}

void entprell(void) {
  uint8_t t = 0;
  _delay_ms(20);
  while (!(PINB & SENSOR)) {
    uint8_t x = 0;
    while (x < 3) {
      if (!(PINB & SENSOR)) {
        x = 0;
        t++;
      }
      else
        x++;
      _delay_ms(10);
      if (t > 66)
        break;
    }
    if (x > 3 || t > 66)
      break;
  }
}

uint8_t taste(void) {
  if (!(PINB & SENSOR)) {
    entprell();
    return 1;
  }
  return 0;
}

uint8_t freigabe_A(void) {
  uint16_t x = 0;
  if (!(PINB & SENSOR)) {
    for (x = 1; x <= EINGABEZEIT; x++) {
      _delay_ms(1);
      if (PINB & SENSOR)
        return 0;
    }
    return 1;
  }
  return 0;
}

void timer0_init(void) {                              // Timer zur Erzeugung eines Ausgangsimpulses
  TCCR0A |= (1 << WGM01 | 1 << WGM00);                // Fast PWM - Mode 7
  TCCR0B |= (1 << WGM02);                             // Fast PWM - Mode 7
  TCCR0A |= (1 << COM0B1);                            // Clear OC0A/OC0B on Compare Match, set OC0A/OC0B at BOTTOM
  TCCR0B |= (1 << CS01 | 1 << CS00);                  // Prescaler 64 = 2000 Ticks pro Sekunde
  OCR0A = 1 + SCHALTDAUER;                            // TOP
  OCR0B = 1;                                          // Vergleichswert
  TIMSK  |= (1 << TOIE0);                             // Overflow Interrupt ermöglichen
}

ISR(TIM0_OVF_vect) {
  TCCR0B &= ~( 1 << CS01 | 1 << CS00 );               // Timer0 ausschalten
}

void timer1_init(void) {                              // Timer zum Messen und zur Ausgabe
  TCCR1 |= (1 << CS12) | (1 << CS11) | (1 << CS10);   // Prescaler 64
  OCR1C  = OCR1A = 200;                               // TOP+Interrupt
  TCCR1 |= ( 1 << CTC1 );                             // CTC1 an
  TIMSK |= ( 1 << OCIE1A );                           // Interrupt ermöglichen
}

ISR (TIM1_COMPA_vect) {
  static uint16_t dauer;
  ueberlauf = ueberlauf + OCR1A;
  if (dauer > 125)
    dauer = dauer - 125;
  else {                                              // dauer=0
    TCCR0B |= ( 1 << CS01 | 1 << CS00 );              // Timer0 einschalten
    dauer = ausgabe;
    if (output == AN) {
      DDRB |= MOTOR;
      counterOut++;
      output = WARTEND;
    }
    if (output == AUS) {
      DDRB &= ~MOTOR;
      output = WARTEND;
    }
  }

  if (dauer < 125)
    OCR1C = OCR1A = dauer;
  else
    OCR1C = OCR1A = 125;
}

ISR (PCINT0_vect)
{
  static uint16_t messung_alt = 0;
  uint16_t x;

  if (!(PINB & SENSOR)) {
    gemessen = 2;
    x = TCNT1 + ueberlauf;
    if (x > messung_alt) {
      messung = x - messung_alt;
      gemessen = 1;
      counterIn++;
    }
    messung_alt = x;
    GIMSK &= ~(1 << PCIE); // Interrupt ausschalten
  }
}

void KalibrierungVoid(void) {
  while (1) {
    SetupModeAktiv = 0;
    EEPROM.put(66, SetupModeAktiv);
    uint8_t Richtung = 0;
    uint16_t Marker = 0;
    uint8_t y = 0;
    uint16_t x = 0;
    uint16_t letzterImpuls = 0;
    volatile uint16_t Radumfang_Counter = (Radumfang * 10) / Kalibrierungswert;
    volatile uint16_t temp = Radumfang_Counter;
    ausgabe = Radumfang_Counter;
    output = AN;
    while (1) {
      if (Radumfang >= 17500) {
        Richtung = 0;
      }
      if (Radumfang <= 9000) {
        Richtung = 1;
      }

      if (taste()) {
        while (!(PINB & SENSOR)) {
          if (y == 1) {
            Marker++;
            _delay_ms(1);
            if (Marker == 5) {
              if (Richtung == 0)
                Richtung = 1;
              else
                Richtung = 0;
            }
            if (Marker == 2000) {
              output = AN;
              EEPROM.put(0, (Radumfang));
              _delay_ms(10);
              ausgabe = (Radumfang * 10) / 66;
              long_delay(500);
              ausgabe = Radumfang_Counter;
              output = AN;
            }
          }
        }
      }

      while ((PINB & SENSOR)) {
        output = AN;
        Marker = 0;
        y = 1;
        x++;
        _delay_ms(5);
        if (Richtung == 0) {
          Radumfang++;
          _delay_ms(10);
        }
        else if (Richtung == 1) {
          Radumfang--;
          _delay_ms(10);
        }
        Radumfang_Counter = (Radumfang * 10) / Kalibrierungswert;
        if (temp < Radumfang_Counter || temp > Radumfang_Counter) {
          output = AUS;
          temp = Radumfang_Counter;
          x = 0;
          output = AN;
          long_delay(10);
        }
        cli();
        ausgabe = temp;
        sei();
        _delay_ms(10);
      }
    }
  }
}


void MaxSpeedVoid(void) {
  SetupModeAktiv = 0;
  EEPROM.put(66, SetupModeAktiv);
  uint8_t Richtung;
  uint16_t Marker = 0;
  uint8_t y = 0;
  uint16_t x;
  uint16_t letzterImpuls = 0;
  ausgabe = Radumfang / max_Tempo;
  if (max_Tempo >= 50)
    Richtung = 0;
  else
    Richtung = 1;
  output = AN;

  while (1) {
    if (taste()) {
      while (!(PINB & SENSOR)) {
        x = 0;
        Marker++;
        _delay_ms(1);
        if (Marker == 2000 && y == 1) {
          output = AUS;
          EEPROM.put(40, max_Tempo);
          if (Richtung == 0)
            Richtung = 1;
          else
            Richtung = 0;
          output = AN;
          ausgabe = Radumfang / max_Tempo;
        }
      }

      while ((PINB & SENSOR)) {
        Marker = 0;
        y = 1;
        x++;
        _delay_ms(1);
        if (Richtung == 1 && x >= 500) {
          max_Tempo++;
          x = 0;
        }
        else if (Richtung == 0 && x >= 500) {
          max_Tempo--;
          x = 0;
        }
        if (max_Tempo == 51)
        {
          max_Tempo = 99;
        }
        if (max_Tempo > 99)
        {
          max_Tempo = 99;
        }
        if (max_Tempo == 98)
        {
          max_Tempo = 50;
        }
        if (max_Tempo < 25)
        {
          max_Tempo = 25;
        }
        output = AN;
        ausgabe = Radumfang / max_Tempo;
      }
    }
  }
}

void SuperPursuitModeVoid(void) {
  while (1) {
    SetupModeAktiv = 0;
    EEPROM.put(66, SetupModeAktiv);
    uint8_t y = 0;
    uint16_t x = 0;
    uint16_t letzterImpuls = 0;
    ausgabe = (Radumfang / (10 + SuperPursuitMode));
    output = AN;
    _delay_ms(1);
    while (1) {
      ausgabe = (Radumfang / (10 + SuperPursuitMode));

      if (taste()) {
        _delay_ms(50);
        if (SuperPursuitMode >= 1)
          SuperPursuitMode = 0;
        else
          SuperPursuitMode = 1;
        output = AUS;
        ausgabe = (Radumfang / (10 + SuperPursuitMode));
        output = AN;
        EEPROM.put(166, SuperPursuitMode);
        _delay_ms(5);
      }

    }
  }
}

int main(void) {
  DDRB &= ~MOTOR;        // MOTORausgang auch zunächst als Eingang
  PORTB |= MOTOR;
  DDRB &= ~SENSOR;
  PORTB |= SENSOR;
  DDRB  &= ~ UNGENUTZT;  // Unbenutzte Pins auf Eingang
  PORTB |= UNGENUTZT;    // mit Pullup
  PRR |= (1 << PRUSI);   // USI abschalten
  ADCSRA &= ~ (1 << ADEN); // ADC abschalten
  PRR |= (1 << PRADC);

  uint16_t temp;
  uint8_t erlaubnis = 0;
  uint16_t timeout = 0;

  timer0_init();
  timer1_init();

  sei();

  //if (max_Tempo == 50)
  //  max_Tempo = 99;

  if (SetupModeAktiv == 1) {
    ausgabe = (Radumfang * 10) / 50;
    output = AN;
    uint8_t y = 0;
    uint16_t x;
    uint16_t letzterImpuls = 0;
    uint16_t Marker = 0;

    if (!(PINB & SENSOR)) {
      while (!(PINB & SENSOR)) {
        Marker++;
        _delay_ms(1);
        if (Marker == 2000) {
          KalibrierungVoid();
        }
      }
    }

    if (Marker == 0) {
      for (x = 1; x <= 10000; x++) {
        _delay_ms(1);
        if (taste()) {
          _delay_ms(50);
          y++;
          letzterImpuls = x;
          while (!(PINB & SENSOR)) {
            Marker++;
            _delay_ms(1);
            if (Marker == 2000) {
              MaxSpeedVoid();
            }
          }
        }
        if (y >= 4) {
          break;
        }
        if ((x - letzterImpuls) >= 2500) {
          break;
        }
      }
      if (y == 3)
        SuperPursuitModeVoid();
      else {
        SetupModeAktiv = 0;
        //counterOut = 0;
        y = 0;
      }
    }
    //counterOut = 0;
    output = AUS;
  }

  erlaubnis = freigabe_A();

  if (freigabe_A() == 0)
    erlaubnis = SuperPursuitMode;

  if (erlaubnis == 1) {
    uint16_t z;
    output = AUS;
    ausgabe = Radumfang / 25;
    output = AN;
    long_delay(1500);
    for (z = 1; z <= 3500; z++) {
      _delay_ms(1);
      if ((PINB & SENSOR)) {
        SetupModeAktiv = 1;
      }
      if (!(PINB & SENSOR)) {
        SetupModeAktiv = 0;
      }
    }
    output = AUS;
  }

  while ((PINB & SENSOR)) {
    EEPROM.put(66, SetupModeAktiv);
    if (!(PINB & SENSOR)) {
      SetupModeAktiv = 0;
      EEPROM.put(66, SetupModeAktiv);
      break;
    }
  }

  while (!(PINB & SENSOR)) {
    SetupModeAktiv = 0;
    EEPROM.put(66, SetupModeAktiv);
    break;
  }

  PCMSK |= 1 << PCINT4; // PB4 verbinden
  GIMSK |= 1 << PCIE; // Pin Change Interrupt ermöglichen

  while (1) {
    timeout++;
    _delay_ms(1);


    if (timeout >= 60000 && counterIn > counterOut && erlaubnis == 1) {                 // Nachführen der ausgelassenen Radumdrehungen nach 90 Sekunden
      if (timeout == 60000) {
        ausgabe = (Radumfang) / 1;
        guteminute = timeout;
        EEPROM.put(80, counterDiff);
      }
      if (guteminute + 30000 == timeout) {
        guteminute = timeout;
        EEPROM.put(80, counterDiff);
      }
      counterDiff = counterIn - counterOut;
      ausgabe = (Radumfang * 10) / 666;
      output = AN;
    }
    if (timeout == 2500) {
      output = AUS;
      messstatus = ERSTMESSUNG;
    }
    if (timeout == RUHEZEIT / 2 && erlaubnis == 1) {
      counterDiff = counterIn - counterOut;
      EEPROM.put(80, counterDiff);
      _delay_ms(5);
    }

    if (timeout >= Radumfang && counterIn <= counterOut) {
      if (erlaubnis == 1) {
        counterDiff = 0;
        EEPROM.put(80, counterDiff);
        _delay_ms(5);
        output = AUS;
      }
      timeout = 0;
      MCUCR |= (1 << SE | 1 << SM1); // Mode Power Down
      sleep_mode();
      MCUCR &= ~(1 << SE);
    }
    if (gemessen == 2) {
      gemessen = 0;
      timeout = 0;
      debounce();
      GIFR |= 1 << PCIF; // Interruptflag löschen
      GIMSK |= 1 << PCIE; // Interrupt wieder einschalten
    }
    if (gemessen == 1) {
      gemessen = 0;
      timeout = 0;
      if (messstatus == ERSTMESSUNG) {
        messstatus = FOLGEMESSUNG;
      }
      else {
        output = AN;
        temp = messung;
        if (temp <= (Radumfang * 10 / min_Tempo) && temp > (Radumfang / (max_Tempo - 1)) && erlaubnis == 1) {
          temp = Radumfang * 100 / (Radumfang * 10 / temp + 1000);
        }
        else if (temp <= (Radumfang / (max_Tempo - 1)) && temp >= (Radumfang / (max_Tempo + 1)) && erlaubnis == 1) {
          temp = Radumfang * 100 / (Radumfang / temp * 2 + 2460);
        }
        cli();
        ausgabe = temp;
        sei();
      }
      debounce();
      GIFR |= 1 << PCIF; // Interruptflag löschen
      GIMSK |= 1 << PCIE; // Interrupt wieder einschalten
    }
  }
  return 0;
}
