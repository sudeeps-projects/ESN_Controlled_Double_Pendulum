/*  ZIPY encoder wiring test - NO MOTOR.
 *
 *  Prints both encoders 10 times a second: the count, the live level of each
 *  signal pin, and how many times each pin changed since the last line.
 *  Turn each shaft slowly by hand and read the "edges" numbers:
 *
 *    edges climb on A and B, count moves   -> encoder and wiring are fine
 *    edges 0/0, pins stuck at 1 1          -> no signal reaches the pins: wire in
 *                                             the wrong hole or row, encoder not
 *                                             powered (check red on VIN), or a dead
 *                                             breadboard strip
 *    edges 0/0, pins stuck at 0 0          -> pin is shorted to GND, or the wire
 *                                             is in a GND hole
 *    edges on one pin only                 -> that one wire (A or B) is the fault;
 *                                             the count only jitters by +/-1
 *
 *  z = zero both counts
 *
 *  POLE ENCODER PINS ARE FIXED: A = 1, B = 2. Pin numbers are the labels printed
 *  on the board, not positions: the top-left pin is GND, then 0, then 1, then 2.
 */

#include <Encoder.h>

const int PWM_PIN = 6, DIR_PIN = 5;
const int POLE_A = 1, POLE_B = 2;     // FIXED: pole A -> pin 1, pole B -> pin 2
const int CART_A = 8, CART_B = 9;
static_assert(POLE_A == 1 && POLE_B == 2, "pole encoder is POLE_A = 1, POLE_B = 2 - do not change");

const float DEG_PER_COUNT = 360.0f / 2400.0f;
const float MM_PER_COUNT  = 40.0f / 2400.0f;

Encoder poleEnc(POLE_A, POLE_B);
Encoder cartEnc(CART_A, CART_B);

const int PINS[4] = { POLE_A, POLE_B, CART_A, CART_B };
int  level[4];
long edges[4];
uint32_t lastPrint = 0;

void setup(){
  pinMode(PWM_PIN, OUTPUT); digitalWrite(PWM_PIN, LOW);   // motor held off
  pinMode(DIR_PIN, OUTPUT); digitalWrite(DIR_PIN, LOW);
  for(int i=0;i<4;i++){ pinMode(PINS[i], INPUT_PULLUP); level[i] = digitalRead(PINS[i]); }
  Serial.begin(115200);
  uint32_t w=millis(); while(!Serial && millis()-w<3000){}
  Serial.println("\nZIPY encoder test. Motor off. Turn each shaft slowly by hand.");
  Serial.printf("Pole encoder A -> pin %d, B -> pin %d.  Cart encoder A -> pin %d, B -> pin %d.\n",
                POLE_A, POLE_B, CART_A, CART_B);
  Serial.println("z = zero counts");
}

void loop(){
  // poll the pins as fast as loop() runs - far faster than the encoders can change
  for(int i=0;i<4;i++){
    int v = digitalRead(PINS[i]);
    if(v != level[i]){ level[i] = v; edges[i]++; }
  }
  while(Serial.available()){
    char c = Serial.read();
    if(c=='z'||c=='Z'){ poleEnc.write(0); cartEnc.write(0); Serial.println("# zeroed"); }
  }
  if(millis()-lastPrint >= 100){
    lastPrint = millis();
    long p = poleEnc.read(), c = cartEnc.read();
    Serial.printf("pole %6ld (%7.1f deg)  pins %d%d  edges %3ld/%-3ld  |  cart %6ld (%7.1f mm)  pins %d%d  edges %3ld/%-3ld\n",
                  p, p*DEG_PER_COUNT, level[0], level[1], edges[0], edges[1],
                  c, c*MM_PER_COUNT,  level[2], level[3], edges[2], edges[3]);
    for(int i=0;i<4;i++) edges[i] = 0;
  }
}
