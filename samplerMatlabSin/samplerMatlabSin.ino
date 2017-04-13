// define the sin wave
static const int maxSamplesNum = 120;

static const int sinWave[maxSamplesNum] = {
  150,155,160,166,171,176,181,186,191,195,200,204,
  209,213,217,221,224,228,231,234,237,239,241,243,
  245,247,248,249,249,250,250,250,249,249,248,247,
  245,243,241,239,237,234,231,228,224,221,217,213,
  209,204,200,195,191,186,181,176,171,166,160,155,
  150,145,140,134,129,124,119,114,109,105,100,96,91,
  87,83,79,76,72,69,66,63,61,59,57,55,53,52,51,51,50,
  50,50,51,51,52,53,55,57,59,61,63,66,69,72,76,79,83,
  87,91,96,100,105,109,114,119,124,129,134,140,145
};

// Volatile Variables, used in the interrupt service routine
volatile int sensorValue1 = 0;
volatile int sensorValue2 = 0;
volatile int sensorValue3 = 0;
// debugging variable
volatile unsigned int deb = 0;
volatile boolean vvv = 0;

// the PWM pin the LED is attached to
int led = 3;   
int debLed = 13;
// element of the LUT to read from       
int ii = 0; 

// debug pin
static int debPin = 13;

void setup() {
  // definition of the analog sensors
  pinMode(A0, INPUT);   
  pinMode(A1, INPUT);    
  pinMode(A2, INPUT);   

  // declare pin 9 to be an output
  pinMode(led, OUTPUT);
  
  // start serial comunication
  Serial.begin(115200);
  
  // whait that the serial connection is established
  while (Serial.available() <= 0) {
    Serial.println('A');   // send a capital A
    delay(300);
  }
  
  // stop interrupts
  cli();
  
  //set timer1 interrupt at 500Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 100Hz increments
  OCR1A = 624;// = (16*10^6) / (100*256) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 256 prescaler
  TCCR1B |= (1 << CS12);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  // allow interrupts
  sei();  
}

void loop() {
  // define the brightness
  int brightness = sinWave[ii];
  
   // set the brightness of pin 9:
  analogWrite(led, brightness);

  // change the valalue of the brightness
  ii = (ii+1) % maxSamplesNum;
 
  delay(30);
}

ISR(TIMER1_COMPA_vect){
  // timer2 interrupt 100Hz 
  Serial.println(deb);
  deb++;
  // read the input on analog pin 0:
  sensorValue1 = analogRead(A0);
  // write data to Serial
  Serial.println(sensorValue1);
  // read the input on analog pin 1:
  sensorValue2 = analogRead(A1);
  // write data to Serial
  Serial.println(sensorValue2);
  // read the input on analog pin 2:
  sensorValue3 = analogRead(A2);
  // write data to Serial
  Serial.println(sensorValue3);

  // debugging
  if (vvv) {
    digitalWrite(debLed,HIGH);
    vvv = 0;
  } else {
    digitalWrite(debLed,LOW);
    vvv = 1;
  }
}
