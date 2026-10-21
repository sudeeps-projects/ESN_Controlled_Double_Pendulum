/*  ZIPY motor-only test - no encoders used.
 *
 *  KEYS (type the letter, press Enter)
 *    w  drive RIGHT for one pulse
 *    l  drive LEFT  for one pulse
 *    x  STOP (s or space also stop)
 *    +  more duty     -  less duty
 *
 *  Every press is one short pulse (PULSE_MS), then the motor stops by itself.
 *  Nothing here knows where the cart is, so it never runs open-ended - press
 *  again to go further. "www" + Enter is NOT three pulses: keys typed while a
 *  pulse is running are ignored, except the stop keys.
 *
 *  If w moves the cart LEFT, set RIGHT_DIR = LOW below. The main sketch does
 *  not care which way is "right" - c and h work the sign out themselves.
 *
 *  Wiring: MD30C PWM -> pin 6, DIR -> pin 5, MD30C GND -> Teensy GND.
 */

const int PWM_PIN = 6, DIR_PIN = 5;
const int POLE_A = 1, POLE_B = 2;     // FIXED: pole A -> pin 1, pole B -> pin 2 (not used here)
static_assert(POLE_A == 1 && POLE_B == 2, "pole encoder is POLE_A = 1, POLE_B = 2 - do not change");

const int      RIGHT_DIR = HIGH;      // DIR level that moves the cart right
const uint32_t PULSE_MS  = 300;       // length of one push
float duty = 0.35f;                   // 0..1, start gentle

bool moving = false;
uint32_t stopAt = 0;

void motor(float d, int dirLevel){
  digitalWrite(DIR_PIN, dirLevel);
  analogWrite(PWM_PIN, (int)(d*4095));
}
void stopMotor(const char* why){
  analogWrite(PWM_PIN, 0);
  if(moving || why[0]){ Serial.print("-- stopped"); Serial.println(why); }
  moving = false;
}

void setup(){
  pinMode(DIR_PIN, OUTPUT); digitalWrite(DIR_PIN, LOW);
  pinMode(PWM_PIN, OUTPUT);
  analogWriteResolution(12); analogWriteFrequency(PWM_PIN, 20000); analogWrite(PWM_PIN, 0);
  Serial.begin(115200);
  uint32_t w=millis(); while(!Serial && millis()-w<3000){}
  Serial.println("\nZIPY motor test. Motor off.");
  Serial.println("w = right   l = left   x = stop   + / - = duty");
  Serial.print("duty "); Serial.print(duty,2); Serial.print("   pulse "); Serial.print(PULSE_MS); Serial.println(" ms");
}

void loop(){
  while(Serial.available()){
    char c = Serial.read();
    if(c=='x'||c=='X'||c=='s'||c=='S'||c==' '){ stopMotor(" by hand"); continue; }
    if(moving) continue;                       // one pulse at a time
    switch(c){
      case 'w': case 'W':
        motor(duty, RIGHT_DIR);  moving=true; stopAt=millis()+PULSE_MS;
        Serial.print("> right  duty "); Serial.println(duty,2); break;
      case 'l': case 'L':
        motor(duty, !RIGHT_DIR); moving=true; stopAt=millis()+PULSE_MS;
        Serial.print("< left   duty "); Serial.println(duty,2); break;
      case '+': case '=':
        duty += 0.05f; if(duty>0.80f) duty=0.80f;
        Serial.print("duty "); Serial.println(duty,2); break;
      case '-': case '_':
        duty -= 0.05f; if(duty<0.10f) duty=0.10f;
        Serial.print("duty "); Serial.println(duty,2); break;
      default: break;                          // Enter, CR, anything else
    }
  }
  if(moving && (int32_t)(millis()-stopAt) >= 0) stopMotor("");
}
