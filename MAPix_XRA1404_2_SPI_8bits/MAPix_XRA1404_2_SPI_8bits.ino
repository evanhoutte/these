// the XRA1404 communicates using SPI, so include the library:
#include <SPI.h>

#define NB_UINT32_BYTE  4
#define NB_INT32_BYTE   4
#define NB_INT16_BYTE  2
#define NB_UINT16_BYTE   2
#define NB_UINT8_EVA   26

#define NB_UINT16_EVA   13 //12 pixels + 1 mean value (on 16 bits)

//#define ECHO_PRINTF
#define START_MSG
#define STOP_MSG


// pins used for the connection with the serializer
const int pwm3 = 3;
const int SyncPin = 5;
const int Clock1MhzPin = 6;
const int chipSelectPin = 10;
const int offsetEva = 230;

SPISettings settingsA(22000000, MSBFIRST, SPI_MODE1);
IntervalTimer Timer;

// eva is the old name for M2APix
uint8_t eva[208]; // size of SPI data packet (12 pixels(16bit)) + 1 mean value (16 bits)) * 8 M2APix = 13*8*2 = 208 octets(Bytes)
uint8_t eva1[NB_UINT16_EVA];
uint8_t eva2[NB_UINT16_EVA];
uint8_t eva3[NB_UINT16_EVA];
uint8_t eva4[NB_UINT16_EVA];
uint8_t eva5[NB_UINT16_EVA];
uint8_t eva6[NB_UINT16_EVA];
uint8_t eva7[NB_UINT16_EVA];
uint8_t eva8[NB_UINT16_EVA];
uint16_t byte8EVA, teva1, teva2, teva3, teva4, teva5, teva6, teva7, teva8;
uint8_t temp;
unsigned long timer = 0, waiting, waiting2, waiting3;
uint8_t i, j = 0;
char *StartMessage = "START";
char *StopMessage = "STOP";
uint8_t ByteRead;
uint8_t StartAsked = 0;
uint8_t StopAsked = 0;

unsigned long DeltaTachy = 0, Ttachy, lastTtachy = 0;

void setup() {
  //Serial.begin(1000000);
  Serial1.begin(3000000);
  // start the SPI library:
  SPI.begin();
  // initalize the  data ready and chip select pins:
  pinMode(2, OUTPUT);
  digitalWriteFast(2, HIGH);
  pinMode(4, INPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(SyncPin, OUTPUT); // same sync pin for all to sample the 8 M2APix at the same time (1000 frame/s)
  pinMode(Clock1MhzPin, OUTPUT); 


  analogWrite(pwm3, 150);
  analogWrite(A14, 0);
  pinMode(chipSelectPin, OUTPUT);
  SPI.beginTransaction(settingsA);
  attachInterrupt(4, tachy, FALLING);

  StartAsked = 0;


#ifdef START_MSG
  while (StartAsked == 0)
  {
    if (Serial1.available() >= strlen(StartMessage))
    {
      // verifiy that the received stream is well the start stream
      for (i = 0; i < strlen(StartMessage); i++)
      {
        ByteRead = Serial1.read();
        if ((uint8_t)StartMessage[i] == ByteRead)
        {
          StartAsked = 1;
        }
      }
    }
    delay(1);
  }
  Serial1.write("START");
#endif
  //digitalWriteFast(SyncPin, HIGH);
  //digitalWriteFast(Clock1MhzPin, LOW);
  Timer.begin(tick, 1000);

}

void tachy()
{
  Ttachy = micros();
  DeltaTachy = ((Ttachy - lastTtachy) + DeltaTachy)/2;
  lastTtachy = Ttachy;
}

void loop()
{
// tick();
  delay(1);
}

void tick() {
  readXRA1404(eva);
  shiftDataTo_uint16_V2( eva, eva1, eva2, eva3, eva4, eva5, eva6, eva7, eva8); //0.640ms avec Uint8 sur serial1 3Mbit/s

  // Send Serial data packet with all 12 pixels of 8 M2APix (EVA) on (each pixels on 8bits)
  Serial1.write((uint8_t)254);  //header to help host to detect begin of transmission
  Serial1.write((uint8_t)round(((float)11.9/((float)DeltaTachy*60/1000000))));
  Serial1.write((const uint8_t*)eva1, 13);
  Serial1.write((const uint8_t*)eva2, 13);
  Serial1.write((const uint8_t*)eva3, 13);
  Serial1.write((const uint8_t*)eva4, 13);
  Serial1.write((const uint8_t*)eva5, 13);
  Serial1.write((const uint8_t*)eva6, 13);
  Serial1.write((const uint8_t*)eva7, 13);
  Serial1.write((const uint8_t*)eva8, 13);

#ifdef STOP_MSG
  if (Serial1.available() >= strlen(StopMessage))
  {
    // verifiy that the received stream is well the stop stream
    for (i = 0; i < strlen(StopMessage); i++)
    {
      ByteRead = Serial1.read();
      if ((uint8_t)StopMessage[i] == ByteRead) StopAsked = 1;
      else StopAsked = 0;
    }
    if (StopAsked)
    {
      Serial1.clear();
      Timer.end();
      Serial1.write("STOP");
      setup();
    }
  }
  else if (Serial1.available() == 3 && Serial1.read() == 254)
  {
    ByteRead = Serial1.read();
    analogWrite(A14, ByteRead);
    ByteRead = Serial1.read();
    analogWrite(pwm3, ByteRead);
  }
#endif
}

void readXRA1404(uint8_t recep[])
{
  digitalWriteFast(SyncPin, LOW); // Start transmission for all M2APix
  for (i = 0; i < 208; i++)
  {
    digitalWriteFast(Clock1MhzPin, HIGH);
    SPI.transfer(0b10000000);
    digitalWriteFast(Clock1MhzPin, LOW);
    recep[i] = SPI.transfer(0);
  }
  digitalWriteFast(SyncPin, HIGH); // End transmission for all M2APix, end of frame
}

// convert parallel bit data received (byte8EVA from 8 M2APix) to serial (eva,...,eva7), 
// simultaneously convert pixels data from 16bits to 8bits (remove useless bits, divide value to delete noise,...)
void shiftDataTo_uint16_V2(uint8_t eva[], uint8_t eva1[], uint8_t eva2[], uint8_t eva3[], uint8_t eva4[], uint8_t eva5[], uint8_t eva6[], uint8_t eva7[], uint8_t eva8[])
{
  for (i = 0; i < 13; i++)
  {
    teva1 = 0;
    teva2 = 0;
    teva3 = 0;
    teva4 = 0;
    teva5 = 0;
    teva6 = 0;
    teva7 = 0;
    teva8 = 0;
    for (j = 5; j < 14; j++)
    {
      byte8EVA = eva[16 * i + j];
      teva1 = teva1 | ((byte8EVA & B1) << (13 - j));
      teva2 = teva2 | ((byte8EVA >> 1 & B1) << (13 - j));
      teva3 = teva3 | ((byte8EVA >> 2 & B1) << (13 - j));
      teva4 = teva4 | ((byte8EVA >> 3 & B1) << (13 - j));
      teva5 = teva5 | ((byte8EVA >> 4 & B1) << (13 - j));
      teva6 = teva6 | ((byte8EVA >> 5 & B1) << (13 - j));
      teva7 = teva7 | ((byte8EVA >> 6 & B1) << (13 - j));
      teva8 = teva8 | ((byte8EVA >> 7 & B1) << (13 - j));
    }
    if (i)
    {
      eva1[i] = (uint8_t) (teva1 - offsetEva);
      eva2[i] = (uint8_t) (teva2 - offsetEva);
      eva3[i] = (uint8_t) (teva3 - offsetEva);
      eva4[i] = (uint8_t) (teva4 - offsetEva);
      eva5[i] = (uint8_t) (teva5 - offsetEva);
      eva6[i] = (uint8_t) (teva6 - offsetEva);
      eva7[i] = (uint8_t) (teva7 - offsetEva);
      eva8[i] = (uint8_t) (teva8 - offsetEva);
    }
    else
    {
      eva1[i] = (uint8_t) teva1;
      eva2[i] = (uint8_t) teva2;
      eva3[i] = (uint8_t) teva3;
      eva4[i] = (uint8_t) teva4;
      eva5[i] = (uint8_t) teva5;
      eva6[i] = (uint8_t) teva6;
      eva7[i] = (uint8_t) teva7;
      eva8[i] = (uint8_t) teva8;
    }
  }

}

