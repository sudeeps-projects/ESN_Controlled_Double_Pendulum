/*  ZIPY drag test - motor + cart encoder, no pole.
 *
 *  Puts the right-hand drag into numbers, so a fix (looser belt, a support
 *  bearing at the encoder end) can be checked rather than guessed at.
 *  START WITH THE CART IN THE MIDDLE OF THE RAIL.
 *
 *  KEYS (type the letter, press Enter)
 *    w  one push RIGHT        l  one push LEFT      (0.3 s at the set duty)
 *    a  find the creep duty each way - the lowest duty that moves the cart
 *    +  more duty             -  less duty
 *    x  STOP (s or space too)
 *
 *  After each push:
 *    > right duty 0.35  moved  41.2 mm  avg  137 mm/s  stuck 0/24
 *  "stuck n/m" counts the 10 ms slices in which the cart did not move while it
 *  was being driven. Smooth running = 0. Jerky = several. Compare right with
 *  left at the same duty.
 *
 *  Keys typed during a push are ignored, except the stop keys.
 *  RIGHT_DIR matches motor_test - change both if w ever goes left.
 */

#include <Encoder.h>

const int PWM_PIN = 6, DIR_PIN = 5;
const int POLE_A = 1, POLE_B = 2;     // FIXED: pole A -> pin 1, pole B -> pin 2 (not used here)
const int CART_A = 8, CART_B = 9;
static_assert(POLE_A == 1 && POLE_B == 2, "pole encoder is POLE_A = 1, POLE_B = 2 - do not change");

const int      RIGHT_DIR = HIGH;      // DIR level that moves the cart right
const uint32_t PULSE_MS  = 300;
const float    MM_PER_COUNT = 40.0f / 2400.0f;
float duty = 0.35f;

Encoder cartEnc(CART_A, CART_B);

void motor(float d, bool right){
  digitalWrite(DIR_PIN, right ? RIGHT_DIR : !RIGHT_DIR);
  analogWrite(PWM_PIN, (int)(d*4095));
}
void motorOff(){ analogWrite(PWM_PIN, 0); }

bool stopKey(){                       // reads the keyboard during a push
  bool s=false;
  while(Serial.available()){
    char c=Serial.read();
    if(c=='x'||c=='X'||c=='s'||c=='S'||c==' ') s=true;
  }
  if(s) motorOff();
  return s;
}
bool waitMs(uint32_t ms){             // delay() that a stop key cuts short
  uint32_t t=millis();
  while(millis()-t<ms){ if(stopKey()) return true; }
  return false;
}

// One timed push. Reports how far it went and how often it stuck on the way.
void pulse(bool right){
  long c0=cartEnc.read(), prev=c0;
  int slices=0, stuck=0;
  motor(duty, right);
  uint32_t t0=millis(), tSlice=t0;
  while(millis()-t0 < PULSE_MS){
    if(stopKey()){ Serial.println("-- stopped by hand"); return; }
    if(millis()-tSlice >= 10){
      tSlice += 10;
      long c=cartEnc.read();
      if(millis()-t0 > 60){           // skip the first 60 ms - it is getting going
        slices++;
        if(labs(c-prev) < 2) stuck++; // under 0.03 mm in 10 ms = not moving
      }
      prev=c;
    }
  }
  motorOff();
  if(waitMs(150)){ Serial.println("-- stopped by hand"); return; }   // let it coast to rest
  long moved=cartEnc.read()-c0;
  float mm=fabsf(moved*MM_PER_COUNT);
  Serial.printf("%s duty %.2f  moved %5.1f mm  avg %4.0f mm/s  stuck %d/%d%s\n",
                right ? "> right" : "< left ", duty, mm, mm/(PULSE_MS/1000.0f),
                stuck, slices, moved==0 ? "   (encoder saw nothing)" : "");
}

// Lowest duty that gets the cart moving one way: short pushes, small steps.
// Returns -1 if stopped by hand, 0 if it never moved.
float creep(bool right){
  for(float d=0.08f; d<=0.601f; d+=0.02f){
    long c0=cartEnc.read();
    motor(d, right);
    bool s=waitMs(150);
    motorOff();
    if(s || waitMs(100)) return -1;
    if(labs(cartEnc.read()-c0)*MM_PER_COUNT > 0.5f) return d;
  }
  return 0;
}

// Three tries each way, alternating so the cart does not wander toward an end.
void creepTest(){
  Serial.println("# finding the creep duty each way - small pushes. Cart should start mid-rail.");
  float l[3], r[3];
  for(int i=0;i<3;i++){
    l[i]=creep(false); if(l[i]<0){ Serial.println("-- stopped by hand"); return; }
    r[i]=creep(true);  if(r[i]<0){ Serial.println("-- stopped by hand"); return; }
  }
  Serial.printf("# creep duty   left  %.2f %.2f %.2f    right  %.2f %.2f %.2f\n",
                l[0],l[1],l[2], r[0],r[1],r[2]);
  float la=(l[0]+l[1]+l[2])/3, ra=(r[0]+r[1]+r[2])/3;
  if(la==0 || ra==0 || l[0]*l[1]*l[2]*r[0]*r[1]*r[2]==0){
    Serial.println("!! a try did not move even at 0.60 - at the end of the rail, jammed,");
    Serial.println("   or the cart encoder is not counting. Centre the cart and try again.");
  } else if(ra > la*1.3f){
    Serial.printf("# right needs %.0f%% more duty than left - real extra drag going right.\n", (ra/la-1)*100);
  } else if(la > ra*1.3f){
    Serial.printf("# left needs %.0f%% more duty than right - extra drag going left.\n", (la/ra-1)*100);
  } else {
    Serial.println("# about the same both ways - no big one-sided drag at creep speed.");
  }
}

void setup(){
  pinMode(DIR_PIN, OUTPUT); digitalWrite(DIR_PIN, LOW);
  pinMode(PWM_PIN, OUTPUT);
  analogWriteResolution(12); analogWriteFrequency(PWM_PIN, 20000); analogWrite(PWM_PIN, 0);
  Serial.begin(115200);
  uint32_t w=millis(); while(!Serial && millis()-w<3000){}
  Serial.println("\nZIPY drag test. Motor off. Put the cart in the MIDDLE of the rail.");
  Serial.println("w = push right   l = push left   a = creep duty both ways   + / - = duty   x = stop");
  Serial.printf("duty %.2f   push %lu ms\n", duty, (unsigned long)PULSE_MS);
}

void loop(){
  while(Serial.available()){
    char c=Serial.read();
    switch(c){
      case 'w': case 'W': pulse(true);  break;
      case 'l': case 'L': pulse(false); break;
      case 'a': case 'A': creepTest();  break;
      case '+': case '=': duty+=0.05f; if(duty>0.80f) duty=0.80f; Serial.printf("duty %.2f\n", duty); break;
      case '-': case '_': duty-=0.05f; if(duty<0.10f) duty=0.10f; Serial.printf("duty %.2f\n", duty); break;
      case 'x': case 'X': case 's': case 'S': case ' ': motorOff(); Serial.println("-- stopped"); break;
      default: break;                  // Enter, CR, anything else
    }
  }
}
