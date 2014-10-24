#define ANALOG_IN 0
const boolean DEBUG = true; // If true, the device will regularly send all internal parameters to the computer
const int BAUD_RATE = 57600;
#define NUMCHANNELS 6
#define HEADERLEN 4
#define PACKETLEN (HEADERLEN + NUMCHANNELS * 2 + 1)

#define FOSC 16000000		// Clock Speed
#define SAMPFREQ 256
#define TIMER0VAL 256 - ((FOSC / 256) / SAMPFREQ)

char const channel_order[] = {0, 1, 2, 3, 4, 5};
volatile int count = 0;
void setup(void)
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = FOSC / 256 ; // set compare match register to desired timer count: 256 Hz
  OCR1A = FOSC / 256 +20 ; // Manual frequency adjustment for frequency (empirical)
  TCCR1B |= (1 << WGM12); // turn on CTC mode:   
  TCCR1B |= (1 << CS10); // runes at clock frequency (multiplier = 1)
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt: 
  sei(); // enable global interrupts:

  delay(500);
  Serial.begin(57600);
}

ISR(TIMER1_COMPA_vect)
{
  uint8_t TXBuf[PACKETLEN];
  memset(TXBuf,0,sizeof(TXBuf));
  TXBuf[0] = 0xA5;	//Sync 0
  TXBuf[1] = 0x5A;	//Sync 1
  TXBuf[2] = 2;		//Protocol version
  TXBuf[3] = count++;		//Packet counter
  
  int channel = 0;
  int val = analogRead(ANALOG_IN);
  TXBuf[channel * 2 + HEADERLEN] = val & 0xFF;
  TXBuf[channel * 2 + 1 + HEADERLEN] = (val >> 8) & 0xFF;
  TXBuf[NUMCHANNELS * 2 + HEADERLEN] = 0b00000001;
  Serial.write(TXBuf, PACKETLEN);
}

void loop() {
  while (1) {  
    __asm__ __volatile__ ("sleep");	// sleep until something happens
  }
}
