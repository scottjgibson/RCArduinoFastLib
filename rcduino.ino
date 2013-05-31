volatile uint8_t prev; // remembers state of input bits from previous interrupt
volatile uint32_t risingEdge[6]; // time of last rising edge for each channel
volatile uint32_t uSec[6]; // the latest measured pulse width for each channel

ISR(PCINT2_vect) { // one or more of pins 2~7 have changed state
  uint32_t now = micros();
  uint8_t curr = PIND; // current state of the 6 input bits
  uint8_t changed = curr ^ prev;
  int channel = 0;
  for (uint8_t mask = 0x04; mask; mask <<= 1) {
    if (changed & mask) { // this pin has changed state
      if (curr & mask) { // +ve edge so remember time
        risingEdge[channel] = now;
      }
      else { // -ve edge so store pulse width
        uSec[channel] = now - risingEdge[channel];
      }
    }
    channel++;
  }
  prev = curr;
}

void setup() {
  Serial.begin(9600);

  for (int pin = 2; pin <= 7; pin++) { // enable pins 2 to 7 as our 6 input bits
    pinMode(pin, INPUT);
  }

  PCMSK2 |= 0xFC; // set the mask to allow those 6 pins to generate interrupts
  PCICR |= 0x04;  // enable interupt for port D
}

void loop() {
  Serial.flush();
  for (int channel = 0; channel < 6; channel++) {
    Serial.print(uSec[channel]);
    Serial.print("\t");
  }
  Serial.println();
}
