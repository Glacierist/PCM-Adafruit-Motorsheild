#include <Adafruit_MotorShield.h>
const byte SIGNAL_PIN = 2;

byte incomingSignalBits [15];
byte lastIncomingSignalBits [15];

long startPulseTime;
long endPulseTime;
long pulseWidth;

int currentSignalPosition = 0;
int receivingFirstSection = 0;
int firstSectionPosition = 0;

struct Positions {
  int b;
  int u;
  int d;
  int l;
  int r;
};
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
Adafruit_DCMotor *M2 = AFMS.getMotor(2);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);

Positions positions = {0, 0, 0, 0, 0};
Positions lastPosition;
void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  Serial.begin(115200);

  EIFR = (1 << INTF0);   //use before attachInterrupt(0,isr,xxxx) to clear interrupt 0 flag
  EIFR = (1 << INTF1);   //use before attachInterrupt(1,isr,xxxx) to clear interrupt 1 flag

  attachInterrupt (digitalPinToInterrupt(SIGNAL_PIN), rising, FALLING);
  attachInterrupt (digitalPinToInterrupt(SIGNAL_PIN), falling, RISING);
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  //M1->setSpeed(100); 
  //M1->run(FORWARD);  
  // Set the speed to start, from 0 (off) to 255 (max speed)

}/*--(end setup )---*/


void loop() {
 
  // Stop interrupts, get a copy of positions and then let interrupts continue
  Positions pos;
  noInterrupts ();
  memcpy (&pos, &positions, sizeof pos);
  interrupts();
  
  if (memcmp(&lastPosition, &pos, sizeof(pos)) != 0) {
      processRemoteCommands(pos);
      lastPosition = pos;
  }
}/* --(end main loop )-- */

/*-----( Declare User-written Functions )-----*/
void processRemoteCommands(Positions pos) {
          
      Serial.print("B: ");
      Serial.println(pos.b);
      Serial.print("U: ");
      Serial.println(pos.u);
      Serial.print("D: ");
      Serial.println(pos.d);
      Serial.print("R: ");
      Serial.println(pos.r);
      Serial.print("L: ");
      Serial.println(pos.l);
      Serial.println("------------------------------");

      if (pos.u == 0 && pos.d == 0 && pos.l == 0 && pos.r == 0) {
        
        Serial.println("stop");
        M1->setSpeed(200); 
        M1->run(RELEASE);
        
        M2->setSpeed(200); 
        M2->run(RELEASE);
        
        M3->setSpeed(200); 
        M3->run(RELEASE);
        
        M4->setSpeed(200); 
        M4->run(RELEASE);
        
      } else if (pos.u > 0) {
        
        Serial.println("up");
        M1->setSpeed(200); 
        M1->run(FORWARD);
        
        M2->setSpeed(200); 
        M2->run(FORWARD);
        
        M3->setSpeed(200); 
        M3->run(FORWARD);
        
        M4->setSpeed(200); 
        M4->run(FORWARD);
      
      } else if (pos.d > 0) {
        
        Serial.println("down");
        M1->setSpeed(200); 
        M1->run(BACKWARD);
        
        M2->setSpeed(200); 
        M2->run(BACKWARD);
        
        M3->setSpeed(200); 
        M3->run(BACKWARD);
        
        M4->setSpeed(200); 
        M4->run(BACKWARD);
        
      } else if (pos.l > 0) {
        
        Serial.println("left");
        M1->setSpeed(200); 
        M1->run(BACKWARD);
        
        M2->setSpeed(200); 
        M2->run(BACKWARD);
        
        M3->setSpeed(200); 
        M3->run(FORWARD);
        
        M4->setSpeed(200); 
        M4->run(FORWARD);
        
      } else if (pos.r > 0) {
        
        Serial.println("right");
        M1->setSpeed(200); 
        M1->run(FORWARD);
        
        M2->setSpeed(200); 
        M2->run(FORWARD);
        
        M3->setSpeed(200); 
        M3->run(BACKWARD);
        
        M4->setSpeed(200); 
        M4->run(BACKWARD);
      }
} //END translateIR

void rising() {
  attachInterrupt (digitalPinToInterrupt(SIGNAL_PIN), falling, RISING);
  startPulseTime = micros();
}

void falling() {
  attachInterrupt (digitalPinToInterrupt(SIGNAL_PIN), rising, FALLING);
  endPulseTime = micros();
  int lastWidth;
  pulseWidth = endPulseTime - startPulseTime;
  lastWidth = pulseWidth;
  if (firstSectionPosition == 14) {
    if (memcmp(&lastIncomingSignalBits, &incomingSignalBits, sizeof(incomingSignalBits)) != 0) {

      // Reset positions to zero bcause we're about to set them all
      positions = { 0, 0, 0, 0, 0 };

      // Write out the bits of the signal
      for(int i = 0; i < 15; i++) {
          Serial.print(incomingSignalBits[i]);
      }
      Serial.println();
   
      if (incomingSignalBits[0] == 1) {
        positions.b = 1;
      }
      
      if (incomingSignalBits[7] == 1) {
        byte val = 0;
        bitWrite(val, 0, incomingSignalBits[3]);
        bitWrite(val, 1, incomingSignalBits[2]);
        positions.u = val;
      }
      
      if (incomingSignalBits[6] == 1) {
        byte val = 0;
        bitWrite(val, 0, incomingSignalBits[3]);
        bitWrite(val, 1, incomingSignalBits[2]);
        positions.d = val;
      }
      
      // If only the button is pressed these both get turned on, so make sure it's only one of them
      if (incomingSignalBits[5] == 1 && incomingSignalBits[4] == 0) {
        byte val = 0;
        bitWrite(val, 0, incomingSignalBits[1]);
        positions.r = val+1;
      }
      
      // If only the button is pressed these both get turned on, so make sure it's only one of them
      if (incomingSignalBits[4] == 1 && incomingSignalBits[5] == 0) {
        byte val = 0;
        bitWrite(val, 0, incomingSignalBits[1]);
        positions.l = val+1;
      }

      // This is the signal when nothing is pressed
      if (incomingSignalBits[4] == 1 && incomingSignalBits[5] == 1 && incomingSignalBits[6] == 1 && incomingSignalBits[7] == 1) {
        positions = { 0, 0, 0, 0, 0 };
      }

      firstSectionPosition = 0;
      currentSignalPosition = 0;
      memcpy(lastIncomingSignalBits, incomingSignalBits, sizeof(incomingSignalBits));
      //    memset(widths, 0, sizeof(widths));

    }
  }
  if (lastWidth > 600) {
    lastWidth = 800;
    if (currentSignalPosition == 0) {
    } else if (currentSignalPosition == 7) {
        currentSignalPosition = 0;
        receivingFirstSection = 1;
    } else if (currentSignalPosition == 10) {
        currentSignalPosition = 0;
        receivingFirstSection = 0;
    } else {
        currentSignalPosition = 0;
        receivingFirstSection = 0;
    }
    firstSectionPosition = 0;
    currentSignalPosition++;
  } else if (lastWidth > 500 && lastWidth < 600) {
    lastWidth = 550;
    if (receivingFirstSection = 1) {
      incomingSignalBits[firstSectionPosition] = 1;
      firstSectionPosition++;
    }
  } else if (lastWidth > 250 && lastWidth < 350) {
    lastWidth = 300;
    if (receivingFirstSection = 1) {
      incomingSignalBits[firstSectionPosition] = 0;
      firstSectionPosition++;
    }
  } else {
    lastWidth = 0;
    firstSectionPosition = 0;
  }
  if (firstSectionPosition > 14) {
    firstSectionPosition = 0;
    receivingFirstSection = 0;
  }

}
