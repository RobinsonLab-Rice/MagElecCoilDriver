
// Teensy PWM pin 17 can be used for 3.3 or 5 volt PWM on FTM1
const char PWMPin = 22;
const char EnPin = 21;
const char TriggerPin = 2;
const char BIFREQ1Pin = 0;
const char BIFREQ2Pin = 1;

const double clockSpeed = 48000000; // Base clock frequency

// Global stimulation enable
boolean Enable = false;

// Target frequency values specified by user
volatile double targetFreq = 60000.0;
volatile double secondFreq = 80000.0;
volatile double targetDuty = 0.5;

// Actual calculate frequency values
double actualFreq = 60000.0;
double actualSecFreq = 80000.0;
double actualDuty = 0.5;
uint16_t roundDuty = 512;
double count = 800;
double duty = 400;

// Target pulse values specified by user
boolean doPulse = true;
volatile double targetPulseRate = 100; // in Hz
volatile double targetPulseDuty = 0.012;

// Actual pulse values
unsigned long pulsePeriod = 10000; // round(1000000 / targetPulseRate); rounding doesn't work on Teensy outside of a function - hard code instead.
unsigned long pulseOnMicros = 120; // round(pulsePeriod * targetPulseDuty);
unsigned long pulseOffMicros = pulsePeriod - pulseOnMicros;
volatile unsigned long pulseMicros = pulseOnMicros;
unsigned long lastPulse = 0ul;

int pulseCount = 0;
int totalPulses = 0;

// Controls for Bi-Frequency pulse mode: ie. each pulse is split into equal parts of the primary and secondary frequencies
boolean doBifreq = false;
boolean pulseDone = false;

// Blink time count
boolean doBlink = false;
unsigned long lastBlink = 0ul;
unsigned long loopMicros = 0ul;

// Listen for pin trigger
boolean doTrigger = false;

// Serial input holder
String message = "";

void setup() {
  pinMode(EnPin,OUTPUT);
  pinMode(PWMPin,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(TriggerPin,INPUT_PULLUP);
  pinMode(BIFREQ1Pin,OUTPUT);
  pinMode(BIFREQ2Pin,OUTPUT);
  analogWriteResolution(10);  // analogWrite value 0 to 1023, or 1024 for high
  analogWriteFrequency(PWMPin, targetFreq); // PWM frequency set for timer associated with specified pin
  message.reserve(65);
  Serial.begin(250000);
}

void loop() {
  loopMicros = micros();
  if (doTrigger && !Enable) {
    if (digitalReadFast(TriggerPin) == LOW) {
      startCmd();
    }
  }
  if (Enable && doPulse && loopMicros - lastPulse >= pulseMicros)  {
    if (digitalReadFast(EnPin))  {
      if (doBifreq && !pulseDone) {
        digitalWriteFast(EnPin,LOW);
        digitalWriteFast(BIFREQ1Pin,LOW);
        digitalWriteFast(BIFREQ2Pin,HIGH);
        analogWriteFrequency(PWMPin, actualSecFreq);
        analogWrite(PWMPin, roundDuty);
        digitalWriteFast(EnPin,HIGH);
        pulseDone = true;
      }
      else  {
        digitalWriteFast(EnPin,LOW);
        digitalWriteFast(BIFREQ2Pin,LOW);
        if (totalPulses > 0)  {
          if (pulseCount++ >= totalPulses)  {
            Enable = false;
            pinMode(PWMPin,OUTPUT);
            digitalWriteFast(PWMPin,LOW);
            Serial.println("Stopped");
          }
        }
        pulseMicros = pulseOffMicros;
      }
    }
    else  {
      digitalWriteFast(BIFREQ1Pin,HIGH);
      if (doBifreq) {
        analogWriteFrequency(PWMPin, actualFreq);
        analogWrite(PWMPin, roundDuty);
        pulseDone = false;
      }
      digitalWriteFast(EnPin,HIGH);
      pulseMicros = pulseOnMicros;
    }
    lastPulse = loopMicros;
  }
  if (doBlink && loopMicros - lastBlink >= 500000) {
    digitalWriteFast(13,!digitalReadFast(13));
    lastBlink = loopMicros;
  }
  if (Serial.available()) {
    message = "";
    for (char ii = 0; ii < 65; ii++)  {
      char c = Serial.read();
      if (c != 10)  {
        message += c;
        if (message.length() == 64) {
          break;
        }
      }
      else {
        break;
      }
    }
    parseMessage();
    message = "";
  }
}

inline void stopCmd() {
  digitalWriteFast(EnPin,LOW);
  Enable = false;
  pinMode(PWMPin,OUTPUT);
  digitalWriteFast(PWMPin,LOW);
  Serial.println("Stopped");
}

inline void startCmd()  {
  if (!Enable)  {
    Enable = true;
    lastPulse = loopMicros;
    pulseCount = 0;
    pulseMicros = pulseOnMicros;
    digitalWriteFast(BIFREQ1Pin,HIGH);
    analogWriteFrequency(PWMPin, actualFreq);
    analogWrite(PWMPin, roundDuty);
    digitalWriteFast(EnPin,HIGH);
    Serial.println("Stimulating");
  }
  else  {
    Serial.println("start disregarded - already running");
  }
}

inline void parseMessage() {
  if (message.startsWith("stop")) {
    stopCmd();
  }
  else if (message.startsWith("start")) {
    startCmd();
  }
  else if (message.startsWith("freq")) {
    int ind = message.indexOf(':',4);
    if (ind > 0) {
      double val = message.substring(ind+1).toFloat();
      if (val > 0 && val < 750000) {
        targetFreq = val;
        calculateVals();
        analogWriteFrequency(PWMPin, actualFreq);
        //Serial.println(actualFreq);
      }
    }
  }
  else if (message.startsWith("seco")) {
    int ind = message.indexOf(':',4);
    if (ind > 0) {
      double val = message.substring(ind+1).toFloat();
      if (val > 0 && val < 750000) {
        secondFreq = val;
        calculateVals();
      }
    }
  }
  else if (message.startsWith("bifreq")) {
    if (doBifreq) {
      doBifreq = false;
      pulseOnMicros *= 2ul;
      pulseOffMicros = pulsePeriod - pulseOnMicros;
      Serial.println("Single frequency pulse mode");
    }
    else  {
      doBifreq = true;
      pulseOnMicros /= 2ul;
      pulseOffMicros = pulsePeriod - pulseOnMicros;
      Serial.println("Bi-Frequency pulse mode");
    }
  }
  else if (message.startsWith("duty")) {
    int ind = message.indexOf(':',4);
    if (ind > 0) {
      double val = message.substring(ind+1).toFloat();
      if (val > 0 && val < 1) {
        targetDuty = val;
        calculateVals();
        if (Enable) {
          analogWrite(PWMPin, round(actualDuty * 1024));
        }
        //Serial.println(actualDuty);
      }
    }
  }
  else if (message.startsWith("pulse")) {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      double val = message.substring(ind+1).toFloat();
      if (val > 0 && val <= 1000)  {
        doPulse = true;
        targetPulseRate = val;
        pulsePeriod = round(1000000 / targetPulseRate);
        pulseOnMicros = round(pulsePeriod * targetPulseDuty);
        pulseOffMicros = pulsePeriod - pulseOnMicros;
        Serial.print("Pulse rate set to ");
        Serial.print(1000000.0 / float(pulsePeriod));
        Serial.println(" Hz.");
      }
      else  {
        doPulse = false;
        Serial.println("Pulsing off - continuous stimulation");
      }
    }
  }
  else if (message.startsWith("pduty")) {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      double val = message.substring(ind+1).toFloat();
      if (val > 0 && val < 1)  {
        targetPulseDuty = val;
        if (doBifreq) {
          targetPulseDuty /= 2;
        }
        pulseOnMicros = round(pulsePeriod * targetPulseDuty);
        pulseOffMicros = pulsePeriod - pulseOnMicros;
        Serial.print("Pulse duty set to ");
        if (doBifreq) {
          Serial.println(float(pulseOnMicros*2) / float(pulsePeriod),3);
        }
        else  {
          Serial.println(float(pulseOnMicros) / float(pulsePeriod),3);
        }
      }
    }
  }
  else if (message.startsWith("count")) {
    int ind = message.indexOf(':',5);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      if (val >= 0 && val <= 1000000) {
        totalPulses = val;
        if (val > 1)  {
          Serial.print(val);
          Serial.println(" pulses per start");
        }
        else if (val > 0) {
          Serial.println("1 pulse per start");
        }
        else  {
          Serial.println("No pulse limit");
        }
      }
    }
  }
  else if (message.startsWith("trigger")) {
    doTrigger = !doTrigger;
    Serial.print("Triggering: ");
    Serial.println(doTrigger);
  }
  else if (message.startsWith("LED")) {
    int ind = message.indexOf(':',3);
    if (ind > 0) {
      int val = message.substring(ind+1).toInt();
      if (val > 0)  {
        if (!doBlink) {
          doBlink = true;
          digitalWriteFast(13,HIGH);
          lastBlink = loopMicros;
        }
      }
      else  {
        doBlink = false;
        digitalWriteFast(13,LOW);
      }
    }
  }
  else if (message.startsWith("check")) {
    Serial.print("Enable: ");
    Serial.println(digitalReadFast(EnPin));
    printVals();
  }
  else if (message.startsWith("ID")) {
    Serial.println("Teensy LC - Coil Driver PWM");
  }
//  else if (message.startsWith("")) {
//    
//  }
  else  {
    Serial.print("Command Unrecognized: <");
    Serial.print(message);
    Serial.println(">");
  }
}

inline void calculateVals()  {
  count = round(clockSpeed / (targetFreq * 2));
  actualFreq = clockSpeed / (count);
  duty = round(targetDuty * count);
  actualDuty = duty / count;
  roundDuty = round(actualDuty * 1024);
  double count2 = round(clockSpeed / (secondFreq * 2));
  actualSecFreq = clockSpeed / count2;
  printVals();
}

void printVals()  {
  Serial.print("Target Frequency: ");
  Serial.print(targetFreq);
  Serial.write(9);
  Serial.print("Actual Frequency: ");
  Serial.print(actualFreq*0.5);
  Serial.write(9);
  Serial.print("Frequency Count: ");
  Serial.println(count);
  Serial.print("Actual Second Frequency: ");
  Serial.println(actualSecFreq*0.5);
  Serial.print("Target Duty: ");
  Serial.print(targetDuty);
  Serial.write(9);
  Serial.print("Actual Duty: ");
  Serial.print(actualDuty);
  Serial.write(9);
  Serial.print("Duty Count: ");
  Serial.println(duty);
  Serial.print("Pulse frequency: ");
  Serial.print(1000000.0 / float(pulsePeriod));
  Serial.write(9);
  Serial.print("Pulse Duty: ");
  if (doBifreq) {
    Serial.println(float(pulseOnMicros*2) / float(pulsePeriod),3);
  }
  else  {
    Serial.println(float(pulseOnMicros) / float(pulsePeriod),3);
  }
  Serial.print("Pulses per start: ");
  if (totalPulses == 0) {
    Serial.println("Unlimited");
  }
  else  {
    Serial.println(totalPulses);
  }
}
