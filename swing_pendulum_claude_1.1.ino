/*  ZIPY cart pole - swing up and balance, version 4.3
 *
 *  4.3 FIXES THE 'h' TROUBLE SEEN AFTER 'l'. The swing-up and balance laws are
 *  untouched - the sign line is exactly as it was in 4.1/4.2.
 *
 *  1. KEYS WERE DROPPED SILENTLY. c, m and h only ran in IDLE. 'd' leaves the
 *     sketch in DECAY until it has counted 13 swings, and if the swing died
 *     sooner it stayed in DECAY for ever. 'l' then hides the status line, the
 *     only place the mode is shown, so 'h' just looked broken. Now every
 *     ignored key says why, any command ends the decay test, and the test
 *     finishes by itself when the swing dies out.
 *
 *  2. THE LOOP "CAUGHT UP" AFTER EVERY BLOCKING COMMAND. h, c and m block for
 *     seconds; afterwards the 500 Hz loop ran every missed tick back to back.
 *     With 'l' on that was one CSV line per tick - thousands at once, which
 *     buried the homing result and could freeze the Serial Monitor - and the
 *     velocities in that burst were nonsense. The loop is now put back on the
 *     clock after anything that blocks, and the log runs at 100 Hz.
 *
 *  3. 'x' DID NOTHING WHILE h, c OR m WAS RUNNING, although homing told you to
 *     press it. They poll for it now. Other keys typed meanwhile are thrown
 *     away, so a second 'h' can no longer queue up a second homing run.
 *
 *  4. HOMING DROVE INTO THE END INSTEAD OF BACK TO THE MIDDLE whenever SIGN_X
 *     was -1. Positive duty is +x by definition (that is how 'c' sets SIGN_X)
 *     and the walk back flipped it a second time. Homing also takes SIGN_X
 *     from its own run now, so it no longer depends on 'c'.
 *
 *  5. 'm' AND 'c' ZEROED THE CART ENCODER, quietly throwing away the origin
 *     'h' had measured - and the documented order runs m after h. Both work
 *     in relative counts now, and 'm' brings the cart back to the middle.
 *     'z' after homing zeroes the pole only.
 *
 *  6. THE DECAY TEST COUNTED HAND TREMOR AS SWINGS. Holding the pole at 90 deg
 *     before letting go, every jitter in thetadot counted as a turning point,
 *     so it could reach 13 "swings" with zero loss and say "plenty of margin"
 *     before the pole was even released. A swing now only counts once the
 *     pole has passed through the bottom.
 *
 *  POLE ENCODER PINS ARE FIXED: A = 1, B = 2, enforced by a static_assert.
 *  Pin 1 is also Serial1 TX, so never call Serial1.begin() in this sketch.
 *
 *  4.1 FIXES A SIGN ERROR THAT WAS IN v3 AND v4. Both drove the cart the WRONG
 *  WAY, which damps the pole instead of pumping it - the rig twitches at about
 *  10 deg and never grows. Simulated against the real plant, the wrong sign
 *  stalls at 10.7 deg; the right one reaches the top in 4 s.
 *
 *  The trap: phi here is NOT the pole's physical deflection from hanging, it is
 *  MINUS it. thetaUp() measures from UPRIGHT, so thetaUp = pi - deflection, and
 *      phi = wrapPi(thetaUp - pi) = -deflection
 *  The law wants cart velocity = -V*sign(deflection) = +V*sign(phi). So the
 *  correct line is (phi > 0) ? +1 : -1. Do not "simplify" this.
 *
 *  v3 got the pumping right: the cart is a velocity source, so energy only
 *  moves when it REVERSES, and the reversal has to land at the bottom of the
 *  swing where |thetadot * cos theta| is biggest. That part stays.
 *
 *  v3 still would not reach the top, for two reasons.
 *
 *  1. IT RAN OUT OF RAIL. v3 held full speed for the whole half swing. As the
 *     swing widens the pole slows down, so each half swing lasts longer and
 *     the cart travels further on every one:
 *
 *         amp  30 deg -> cart needs +/- 97 mm
 *         amp 120 deg -> cart needs +/-131 mm
 *         amp 170 deg -> cart needs +/-232 mm   <- you have +/-230
 *
 *     It hits the end exactly on the swing that would have finished the job.
 *     Worse, v3's soft wall tapered the drive linearly from the CENTRE, so it
 *     was throttling the pump from about 60 mm out. That was a bug.
 *
 *     v4 only drives in a window around the bottom, then spends the rest of
 *     the swing putting the cart back in the middle. The pole barely notices
 *     the recentring because it happens near the turning points, where
 *     thetadot is almost zero and moving the cart costs almost nothing. Peak
 *     excursion drops from +/-232 mm to +/-186 mm, and it keeps about 65% of
 *     the energy per pump - one or two extra swings, in exchange for finishing.
 *
 *  2. FRICTION MIGHT BE EATING IT. A 600 p/r encoder has real seal drag, and
 *     every swing loses some energy to it. If the loss per swing is bigger
 *     than the gain per pump, NO control law will ever get this pole up, and
 *     everything else is wasted effort.
 *
 *     Press 'd' and it measures that directly: let the pole swing freely, and
 *     it reports energy lost per half swing and compares it with the energy a
 *     pump can deliver. That is the go/no-go test for the whole machine.
 *
 *  ORDER:  z  (pole hanging still)  ->  c  ->  h  ->  m  ->  d  ->  u
 *
 *  KEYS
 *    z  zero          c  calibrate signs      h  home + set the fence
 *    m  measure the cart
 *    d  free-decay test (motor stays OFF - you push the pole by hand)
 *    u  swing up and balance                  b  balance only
 *    l  CSV log       x  STOP                 p  print numbers
 */

#include <Encoder.h>

const int PWM_PIN = 6, DIR_PIN = 5;
const int POLE_A = 1, POLE_B = 2;     // FIXED: pole A -> pin 1, pole B -> pin 2
const int CART_A = 8, CART_B = 9;
static_assert(POLE_A == 1 && POLE_B == 2, "pole encoder is POLE_A = 1, POLE_B = 2 - do not change");

const float M_PER_COUNT   = 0.040f / 2400.0f;
const float RAD_PER_COUNT = 2.0f * PI / 2400.0f;

const float L = 0.402f, G = 9.81f;
const float E_BOTTOM = -G * L;
const float D2R = PI / 180.0f;

float SIGN_X = +1.0f, SIGN_THETA = +1.0f;
float A_MAX = 2.2f, V_MAX = 0.367f, FRICTION_FF = 0.08f;

// ---- swing-up ------------------------------------------------------------
float DUTY_SWING = 0.95f;
const float PHI_DEAD  = 0.035f;   // hold direction within 2 deg of the bottom
const float SENS_GATE = 0.35f;    // pump while |wF*cos th| is above this
                                  // fraction of its recent peak; recentre below
float KX_R = 3.2f, KV_R = 1.1f;   // recentring gains, in DUTY units
const float E_HANDOVER  = -0.80f;
const float TH_HANDOVER = 35.0f;

// ---- balance -------------------------------------------------------------
float KX = 3.39f, KV = 4.46f, KTH = 37.17f, KW = 6.75f;
const float GIVEUP_DEG = 40.0f;
const float DUTY_MAX   = 0.98f;

// ---- virtual boundaries --------------------------------------------------
// The cart is never allowed within X_MARGIN of the physical end of its travel.
// X_END is the half travel to that end; press 'h' and it measures YOUR rail
// instead of trusting the default, because 'z' zeroes the cart encoder wherever
// the cart happens to be sitting - a fence measured from a bad origin is worse
// than no fence at all.
float X_END = 0.230f;              // half travel to the mechanical end
const float X_MARGIN = 0.040f;     // the virtual boundary
float X_MAX = X_END - X_MARGIN;    // 190 mm: the cart never goes past this
const float X_BRAKE = 0.030f;      // start easing off this far from the fence
const float FENCE_PUSH = 0.45f;    // duty used to shove it back if it gets out
bool homed = false;
const uint32_t LOOP_HZ = 500;
const int LOG_EVERY = 5;           // CSV on every 5th tick = 100 Hz. 1 = full 500 Hz

Encoder cartEnc(CART_A, CART_B);
Encoder poleEnc(POLE_A, POLE_B);

enum Mode { IDLE, SWINGUP, BALANCE, DECAY };
Mode mode = IDLE;

float xF=0, vF=0, thF=0, wF=0, xPrev=0, thPrev=0, lastDuty=0;
uint32_t nextLoop=0, lastPrint=0, t0=0;
bool logging=false, pumping=false;
int logTick=0;

int dirHold=+1, pumps=0, flips=0, retries=0;
float phiPrev=0, eAtPump=0, lastGain=0, sensPk=0.5f;
uint32_t signCheckAt=0, kickUntil=0, bestAt=0;
float eAtCheck=0, eBest=-9;
float ampDeg=0, ampPeak=0, halfPeriodMs=0, wPrevSign=0, xExcursion=0;
uint32_t lastCross=0;

// free-decay test
int   decayN=0;
float decayPeak=0, decayLast=0, decayFirst=0, decayBigA=0, decayLoss=0;
float decayPhiPrev=0;
bool  decayCrossed=false;
uint32_t decayStartAt=0, decayTurnAt=0;

float wrapPi(float a){ while(a>PI)a-=2*PI; while(a<-PI)a+=2*PI; return a; }
float thetaUp(){ return SIGN_THETA * wrapPi(poleEnc.read()*RAD_PER_COUNT - PI); }
float cartX()  { return SIGN_X * cartEnc.read() * M_PER_COUNT; }
float energy(float th,float w){ return (L*L/6.0f)*w*w + (G*L/2.0f)*(cosf(th)-1.0f); }

// energy a bottom reversal can deliver at swing amplitude `amp` (rad)
float pumpGain(float amp){
  float wBot = sqrtf(3.0f*G*(1.0f-cosf(amp))/L);
  return 0.65f * (L/2.0f) * (2.0f*V_MAX) * wBot;   // 0.65 = windowed, not full
}

// Clamp any duty command so the cart cannot pass the virtual boundary. Only
// the OUTBOUND command is damped - coming home is never slowed, so the cart can
// always escape a corner it has got itself into.
float fence(float duty, float x){
  float d = X_MAX - fabsf(x);
  if(d <= 0.0f) return (x > 0.0f) ? -FENCE_PUSH : FENCE_PUSH;   // outside: push back
  if(d < X_BRAKE){
    bool outbound = (x > 0.0f && duty > 0.0f) || (x < 0.0f && duty < 0.0f);
    if(outbound) duty *= d / X_BRAKE;
  }
  return duty;
}

void drive(float d){
  if(d> DUTY_MAX) d= DUTY_MAX;
  if(d<-DUTY_MAX) d=-DUTY_MAX;
  lastDuty=d;
  digitalWrite(DIR_PIN, d>=0?HIGH:LOW);
  analogWrite(PWM_PIN,(int)(fabsf(d)*4095));
}
void halt(const char*w){ drive(0); mode=IDLE; Serial.print("-- stopped: "); Serial.println(w); }

const char* modeName(){
  return mode==SWINGUP?"swing":mode==BALANCE?"BAL":mode==DECAY?"decay":"idle";
}

// c, m and h block for seconds and used to ignore the keyboard until they were
// done, so 'x' did nothing while the cart was moving. They poll this instead.
// Any other key typed meanwhile is thrown away - a second 'h' pressed because
// the first one seemed to do nothing must not queue up another homing run.
bool stopPressed(){
  bool stop=false;
  while(Serial.available()){
    char c=Serial.read();
    if(c=='x'||c=='X') stop=true;
    else if(c>' '){ Serial.print("# ignored '"); Serial.print(c); Serial.println("' - busy, x stops"); }
  }
  if(stop) drive(0);
  return stop;
}
bool waitMs(uint32_t ms){            // delay() that x can cut short
  uint32_t t=millis();
  while(millis()-t<ms){ if(stopPressed()) return true; delay(1); }
  return stopPressed();
}

// Put the 500 Hz loop back on the clock after anything that blocked it.
// Without this it ran every missed tick back to back to catch up - with 'l' on
// that was thousands of CSV lines at once, burying the homing result - and the
// velocity filters saw the jump to the new origin as a huge spike.
void resync(){
  xPrev=xF=cartX(); thPrev=thF=thetaUp();
  vF=wF=0; wPrevSign=0;
  nextLoop=micros();
}

// Drive back to x = 0. Positive duty is +x whatever SIGN_X is - 4.2 flipped the
// duty by SIGN_X a second time here, which drove the cart INTO the end instead
// of home whenever SIGN_X was -1. Returns true if x was pressed.
bool centreCart(uint32_t timeoutMs){
  const float cd = fmaxf(0.26f, FRICTION_FF + 0.06f);
  uint32_t t0c=millis();
  while(fabsf(cartX()) > 0.004f && millis()-t0c < timeoutMs){
    drive(cartX() > 0 ? -cd : cd);
    if(waitMs(10)) return true;
  }
  drive(0);
  if(fabsf(cartX()) > 0.010f){
    Serial.print("!! could not get back to the middle - cart at ");
    Serial.print(cartX()*1000,0); Serial.println(" mm");
  }
  return false;
}

void printNums(){
  Serial.print("SIGN_X ");Serial.print(SIGN_X,0);
  Serial.print("  SIGN_THETA ");Serial.print(SIGN_THETA,0);
  Serial.print("  A_MAX ");Serial.print(A_MAX,2);
  Serial.print("  V_MAX ");Serial.print(V_MAX,3);
  Serial.print("  FRIC ");Serial.println(FRICTION_FF,3);
}

void startSwing(){
  mode=SWINGUP;
  dirHold=+1; pumps=0; flips=0;
  kickUntil=millis()+200; signCheckAt=millis()+3000;
  eAtCheck=energy(thetaUp(),0); eBest=eAtCheck; bestAt=millis();
  eAtPump=eAtCheck; lastGain=0; sensPk=0.5f; xExcursion=0;
  Serial.println("# pumping: drive away from the pole near the bottom, recentre near the ends.");
}

void calibrate(){
  Serial.println("# calibrating - cart moves ~40 mm. Pole hanging still. x stops it.");
  // The cart is measured in relative counts: zeroing it here threw away the
  // origin that 'h' had measured. The pole IS zeroed - it is hanging still.
  poleEnc.write(0);
  if(waitMs(300)){ halt("by hand, during calibrate"); return; }
  long c0=cartEnc.read(), p0=poleEnc.read();
  drive(0.55f);
  bool stopped=waitMs(220);
  drive(0);
  long dx=cartEnc.read()-c0;
  if(stopped || waitMs(120)){ halt("by hand, during calibrate"); return; }
  long dp=poleEnc.read()-p0;
  if(waitMs(900)){ halt("by hand, during calibrate"); return; }
  if(dx==0){ Serial.println("!! cart encoder did not move. Belt slipping, or wiring."); mode=IDLE; return; }
  if(labs(dp)<4) Serial.println("!! pole barely moved - is the hub set screw fitted?");
  SIGN_X     = (dx>0)? +1.0f : -1.0f;
  SIGN_THETA = (dp<0)? -1.0f : +1.0f;
  Serial.print("# cart "); Serial.print(dx*M_PER_COUNT*1000,1);
  Serial.print(" mm, pole "); Serial.print(dp*RAD_PER_COUNT/D2R,1); Serial.println(" deg");
  printNums(); mode=IDLE;
}

void measure(){
  Serial.println("# measuring - the cart will run most of the rail. x stops it.");
  // Relative counts throughout: 4.2 zeroed the encoder here, which threw away
  // the origin 'h' had measured. Once homed, every move heads for the middle.
  if(waitMs(200)){ halt("by hand, during measure"); return; }
  float fric=0;
  float toward = (homed && cartX()>0) ? -1.0f : +1.0f;
  for(float d=0.03f; d<0.60f; d+=0.01f){
    long c0=cartEnc.read(); drive(toward*d);
    if(waitMs(130)){ halt("by hand, during measure"); return; }
    if(labs(cartEnc.read()-c0)>20){ fric=d; break; }
  }
  drive(0);
  if(waitMs(600)){ halt("by hand, during measure"); return; }
  float dir = homed ? ((cartX()>0) ? -1.0f : +1.0f) : (SIGN_X>0 ? +1.0f : -1.0f);
  float vPk=0,aEst=0; long cStart=cartEnc.read(), cP=cStart; uint32_t tP=micros();
  drive(dir*0.95f);
  uint32_t ts=micros();
  while(micros()-ts<500000UL){
    delayMicroseconds(3000);
    if(stopPressed()){ halt("by hand, during measure"); return; }
    uint32_t tn=micros(); long cn=cartEnc.read();
    float v=fabsf((cn-cP)*M_PER_COUNT)/((tn-tP)*1e-6f);
    cP=cn; tP=tn;
    uint32_t el=tn-ts;
    if(el>20000UL && el<60000UL && v>0) aEst=v/(el*1e-6f);
    if(v>vPk) vPk=v;
    bool atFence = homed ? (dir*cartX() > X_MAX - 0.010f)                       // stay inside the fence
                         : (fabsf((cn-cStart)*M_PER_COUNT) > X_MAX - 0.010f);
    if(atFence) break;
  }
  drive(0);
  if(fric>0) FRICTION_FF=fric;
  if(vPk>0.05f) V_MAX=vPk;
  if(aEst>0.3f) A_MAX=aEst;
  Serial.println();
  Serial.print("# creep duty      "); Serial.println(FRICTION_FF,3);
  Serial.print("# top speed       "); Serial.print(V_MAX,3); Serial.println(" m/s");
  Serial.print("# accel from rest "); Serial.print(A_MAX,2); Serial.println(" m/s^2");
  Serial.print("# pump delivers   "); Serial.print(pumpGain(150*D2R),3);
  Serial.println(" J/kg per reversal at a 150 deg swing");
  Serial.print("# the pole needs  "); Serial.print(-E_BOTTOM,2); Serial.println(" J/kg in total");
  if(V_MAX < 0.30f){
    Serial.println("!! low top speed. Energy per pump scales directly with it -");
    Serial.println("   a 40T pulley on the motor doubles it (04_pulley_large_40t.stl).");
  }
  printNums();
  if(homed){
    if(waitMs(300) || centreCart(8000)){ halt("by hand, while centring after measure"); return; }
    Serial.print("# back in the middle, cart at "); Serial.print(cartX()*1000,0); Serial.println(" mm");
  } else Serial.println("!! not homed - the cart was left where the run ended. Press h.");
  mode=IDLE;
}

// Creep to each end, note where the cart stops, and make the midpoint zero.
// This is what makes the fence trustworthy: it measures the real travel rather
// than assuming it, and it puts the origin in the actual middle of the rail.
void home(){
  Serial.println();
  Serial.println("# HOMING. The cart will creep to each end and touch it gently.");
  Serial.println("# Keep hands clear. Press x if anything sounds wrong.");
  const float HD = 0.22f;
  long ends[2];
  for(int i=0;i<2;i++){
    drive(i==0 ? -HD : +HD);
    long last = cartEnc.read();
    uint32_t still = millis(), t0h = millis();
    while(millis()-t0h < 15000){
      if(waitMs(20)){ halt("by hand, during homing - fence unchanged"); return; }
      long c = cartEnc.read();
      if(labs(c-last) > 3){ last = c; still = millis(); }
      else if(millis()-still > 250) break;        // not moving = at the end
    }
    drive(0);
    if(waitMs(450)){ halt("by hand, during homing - fence unchanged"); return; }
    ends[i] = cartEnc.read();
  }
  long  span = ends[1]-ends[0];       // raw counts, -duty end to +duty end
  long  mid  = (ends[0]+ends[1])/2;
  float half = fabsf(span * M_PER_COUNT) / 2.0f;
  if(half < 0.10f || half > 0.45f){
    Serial.print("!! measured half travel "); Serial.print(half*1000,0);
    Serial.println(" mm - that cannot be right. Belt slipping, or it jammed.");
    Serial.println("   If the cart never left the end, 0.22 duty is below its creep duty.");
    Serial.println("   Fence left at its default. Fix the mechanics and try again.");
    mode = IDLE; return;
  }
  // Positive duty is +x by definition. This run just drove the whole rail with
  // known duty signs, so it gets SIGN_X for free and does not need 'c' first.
  float sx = (span > 0) ? +1.0f : -1.0f;
  if(sx != SIGN_X){
    Serial.print("# SIGN_X set to "); Serial.print(sx,0);
    Serial.println(" by homing (c not run, or it got it wrong)");
    SIGN_X = sx;
  }
  cartEnc.write(cartEnc.read() - mid);   // the cart is at the + end right now
  X_END = half;
  X_MAX = X_END - X_MARGIN;
  homed = true;
  Serial.print("# travel +/-"); Serial.print(X_END*1000,0);
  Serial.print(" mm   fence at +/-"); Serial.print(X_MAX*1000,0);
  Serial.print(" mm   (");  Serial.print(X_MARGIN*1000,0);
  Serial.println(" mm clear of each end)");
  // walk back to the middle
  if(centreCart(8000)){ halt("by hand, while centring - homing result kept"); return; }
  Serial.print("# centred, cart at "); Serial.print(cartX()*1000,0); Serial.println(" mm");
  mode = IDLE;
}

void startDecay(){
  mode=DECAY; decayN=0; decayPeak=0; decayLast=0; decayFirst=0; decayBigA=0;
  decayPhiPrev=0; decayCrossed=false; decayStartAt=decayTurnAt=millis();
  drive(0);
  Serial.println();
  Serial.println("# FREE DECAY TEST. The motor is OFF and stays off.");
  Serial.println("# Lift the pole to about 90 deg and let go. Hold the cart still if you can.");
  Serial.println("# It will time the swings and tell you if friction is beating you.");
}

// Called after 13 turns, or earlier if the swing dies out first - 4.2 only
// ever finished at 13, so a pole that stopped sooner left it in DECAY for ever.
void finishDecay(){
  int n = decayN-1;                       // half swings between the turns
  decayLoss = (decayFirst-decayLast)/n;
  float gain = pumpGain(decayBigA);
  Serial.println();
  Serial.print("# over "); Serial.print(n); Serial.println(" half swings");
  Serial.print("# lost per half swing   "); Serial.print(decayLoss,4); Serial.println(" J/kg");
  Serial.print("# a pump can deliver    "); Serial.print(gain,4); Serial.println(" J/kg at that amplitude");
  if(gain > decayLoss*3.0f){
    Serial.println("# VERDICT: plenty of margin. Friction is not your problem - press u.");
  } else if(gain > decayLoss*1.3f){
    Serial.println("# VERDICT: it will climb, but slowly, and may stall near the top.");
    Serial.println("#   A 40T motor pulley doubles the gain and fixes the margin.");
  } else {
    Serial.println("# VERDICT: friction is eating the pump. NO control law can fix this.");
    Serial.println("#   Look at the encoder's drag, the hub rubbing the bracket, and the");
    Serial.println("#   cable pulling on the pole. Fix the mechanics before touching code.");
  }
  mode=IDLE;
}

void handleKey(char c){
  if(c=='x'||c=='X'){ halt("by hand"); return; }
  if(c=='l'||c=='L'){
    logging=!logging; logTick=0;
    if(logging){
      Serial.print("# logging on, "); Serial.print(LOOP_HZ/LOG_EVERY);
      Serial.print(" Hz, mode "); Serial.print(modeName());
      Serial.println(homed ? ", homed" : ", NOT homed");
      Serial.println("# t_ms,x_mm,th_deg,w_dps,E,duty");
    } else Serial.println("# logging off");
    return;
  }
  if(c<=' ') return;                      // CR / LF from the Serial Monitor
  if(mode==SWINGUP || mode==BALANCE){
    Serial.print("!! '"); Serial.print(c); Serial.print("' ignored - ");
    Serial.print(modeName()); Serial.println(" is running, press x first");
    return;
  }
  // DECAY has the motor off, so starting anything else simply ends the test
  if(mode==DECAY && strchr("cCmMhHuUbB", c)){ Serial.println("# decay test abandoned"); mode=IDLE; }
  switch(c){
    case 'z': case 'Z':
      poleEnc.write(0);
      if(homed) Serial.println("# pole zeroed - cart origin kept from homing");
      else { cartEnc.write(0); Serial.println("# zeroed"); }
      resync(); break;
    case 'c': case 'C': calibrate(); resync(); break;
    case 'm': case 'M': measure();   resync(); break;
    case 'h': case 'H': home();      resync(); break;
    case 'd': case 'D': startDecay(); break;
    case 'u': case 'U':
      if(!homed) Serial.println("!! not homed - press h first, or the fence is only a guess");
      startSwing(); break;
    case 'b': case 'B': mode=BALANCE; retries=0; Serial.println("# balancing"); break;
    case 'p': case 'P': printNums(); break;
    default: Serial.print("!! unknown key '"); Serial.print(c); Serial.println("'"); break;
  }
}

void setup(){
  pinMode(CART_A,INPUT_PULLUP); pinMode(CART_B,INPUT_PULLUP);
  pinMode(POLE_A,INPUT_PULLUP); pinMode(POLE_B,INPUT_PULLUP);
  pinMode(DIR_PIN,OUTPUT); digitalWrite(DIR_PIN,LOW);
  pinMode(PWM_PIN,OUTPUT);
  analogWriteResolution(12); analogWriteFrequency(PWM_PIN,20000); analogWrite(PWM_PIN,0);
  Serial.begin(115200);
  uint32_t w=millis(); while(!Serial && millis()-w<3000){}
  Serial.println("\nZIPY swing-up v4.3. Motor off.");
  Serial.print("Pole encoder A -> pin "); Serial.print(POLE_A);
  Serial.print(", B -> pin "); Serial.println(POLE_B);
  Serial.println("Order:  z  ->  c  ->  h  ->  m  ->  d  ->  u");
  Serial.print("Virtual boundary: +/-"); Serial.print(X_MAX*1000,0);
  Serial.print(" mm (assumed). Press h to measure your rail.\n");
  printNums();
  nextLoop=micros(); t0=millis();
}

void loop(){
  while(Serial.available()) handleKey(Serial.read());

  if((int32_t)(micros()-nextLoop)<0) return;
  nextLoop += 1000000UL/LOOP_HZ;
  // more than 10 ticks behind (a slow serial host can do it): skip ahead
  // rather than run the missed ticks back to back
  if((int32_t)(micros()-nextLoop) > (int32_t)(10*1000000UL/LOOP_HZ)) nextLoop=micros();
  const float dt = 1.0f/LOOP_HZ;

  float x=cartX(), th=thetaUp();
  const float alpha = dt/(0.020f+dt);
  vF += alpha*((x-xPrev)/dt - vF);
  wF += alpha*(wrapPi(th-thPrev)/dt - wF);
  xPrev=x; thPrev=th; xF=x; thF=th;

  float E   = energy(thF,wF);
  float phi = wrapPi(thF - PI);
  if(fabsf(xF) > xExcursion) xExcursion = fabsf(xF);

  float adeg = fabsf(phi)/D2R;
  if(adeg>ampPeak) ampPeak=adeg;
  if(wPrevSign<=0 && wF>0){
    uint32_t now=millis();
    if(lastCross) halfPeriodMs = 0.6f*halfPeriodMs + 0.4f*(now-lastCross);
    lastCross=now; ampDeg=ampPeak; ampPeak=0;
  }

  float duty=0;

  // ======================= FREE DECAY ====================================
  if(mode==DECAY){
    drive(0);
    // A half swing runs from the bottom out to a turning point. Restart the
    // peak at every pass through the bottom, and only count a turn once the
    // pole has passed it - otherwise hand tremor while you hold it at 90 deg
    // counts as a stream of zero-loss "swings".
    if(phi*decayPhiPrev < 0){ decayCrossed=true; decayPeak=0; }
    decayPhiPrev=phi;
    if(fabsf(phi) > decayPeak) decayPeak = fabsf(phi);
    bool turned = (wPrevSign>0 && wF<=0) || (wPrevSign<0 && wF>=0);
    if(turned && decayCrossed && decayPeak > 0.12f){
      float Ed = (G*L/2.0f)*(1.0f - cosf(decayPeak));    // energy above hanging
      if(decayN==0){ decayFirst=Ed; decayBigA=decayPeak; }
      else {
        Serial.print("  swing "); Serial.print(decayN);
        Serial.print("  amp "); Serial.print(decayPeak/D2R,1);
        Serial.print(" deg   E "); Serial.print(Ed,3);
        Serial.print("   lost "); Serial.println(decayLast-Ed,4);
      }
      decayLast=Ed; decayN++;
      decayPeak=0; decayCrossed=false; decayTurnAt=millis();
      if(decayN==13) finishDecay();
    } else if(decayN>0 && millis()-decayTurnAt > 4000){
      // the swing died before 13 turns - report what it has
      if(decayN>=4) finishDecay();
      else {
        Serial.print("# swing died after "); Serial.print(decayN);
        Serial.println(" turns - too few to judge. Lift it higher and press d again.");
        mode=IDLE;
      }
    } else if(decayN==0 && millis()-decayStartAt > 60000){
      Serial.println("# no swing seen in 60 s - decay test ended. Press d to retry.");
      mode=IDLE;
    }
    wPrevSign=wF;

  // ======================= SWING UP ======================================
  } else if(mode==SWINGUP){
    // how much the pole cares about cart motion right now
    float sens = fabsf(wF*cosf(thF));
    if(sens>sensPk) sensPk=sens;
    sensPk *= 0.9994f;                      // decays over ~3 s, tracks the growth
    if(sensPk<0.05f) sensPk=0.05f;          // a high floor here stops it ever starting

    // Below about 55 deg the cart hardly travels, so there is no rail to save:
    // pump flat out and get the swing going. Only start conserving once it is wide.
    bool conserve = (E > -3.2f);
    if(!conserve || sens > SENS_GATE*sensPk){
      // --- near the bottom: this is where energy moves. Drive away from the pole.
      // phi = -(deflection from hanging), see the note at the top of the file
      if(fabsf(phi)>PHI_DEAD) dirHold = (phi>0)? +1 : -1;
      duty = DUTY_SWING*dirHold;
      pumping=true;
    } else {
      // --- near a turning point: thetadot is tiny, so moving the cart is almost
      //     free. Spend that window putting it back in the middle.
      duty = -(KX_R*xF + KV_R*vF);
      pumping=false;
    }

    if(millis()<kickUntil){ duty=DUTY_SWING; pumping=true; }

    if(phi*phiPrev<0 && fabsf(wF)>0.4f){ pumps++; lastGain=E-eAtPump; eAtPump=E; }
    phiPrev=phi;

    duty = fence(duty, xF);

    if(millis()>signCheckAt){
      if(flips<2 && E<eAtCheck+0.10f){
        SIGN_THETA=-SIGN_THETA; thPrev=-thPrev; wF=-wF; phiPrev=-phiPrev;
        flips++; Serial.println("# energy not climbing - flipped SIGN_THETA");
      }
      eAtCheck=E; signCheckAt=millis()+3000;
    }
    if(E>eBest+0.05f){ eBest=E; bestAt=millis(); }
    if(millis()-bestAt>12000){
      Serial.print("# stalled at amp "); Serial.print(ampDeg,0);
      Serial.print(" deg, E "); Serial.print(eBest,2);
      Serial.print(", cart reached "); Serial.print(xExcursion*1000,0); Serial.println(" mm");
      halt("no progress in 12 s - run 'd', then read the notes at the bottom");
    }

    if(E>E_HANDOVER && fabsf(thF)/D2R<TH_HANDOVER){
      mode=BALANCE; retries=0;
      Serial.print("# caught at "); Serial.print(thF/D2R,1);
      Serial.print(" deg, "); Serial.print(wF/D2R,0);
      Serial.print(" dps, after "); Serial.print(pumps); Serial.println(" reversals");
    }
    drive(duty);
    wPrevSign=wF;

  // ======================= BALANCE =======================================
  } else if(mode==BALANCE){
    float aCmd = KX*xF + KV*vF + KTH*thF + KW*wF;
    duty = aCmd/A_MAX;
    float b=aCmd; if(b>1)b=1; if(b<-1)b=-1;
    duty += FRICTION_FF*b;
    duty = fence(duty, xF);
    if(fabsf(xF) > X_END) halt("cart past the mechanical end - check the homing");
    else if(fabsf(thF)/D2R>GIVEUP_DEG){
      if(retries<4){ retries++; Serial.print("# dropped it - try "); Serial.print(retries+1); Serial.println(); startSwing(); }
      else halt("could not catch it after 5 tries");
    } else drive(duty);
    wPrevSign=wF;

  } else { drive(0); wPrevSign=wF; }

  if(logging){
    if(++logTick >= LOG_EVERY){
      logTick=0;
      Serial.print(millis()-t0); Serial.print(',');
      Serial.print(xF*1000,1);   Serial.print(',');
      Serial.print(thF/D2R,2);   Serial.print(',');
      Serial.print(wF/D2R,1);    Serial.print(',');
      Serial.print(E,3);         Serial.print(',');
      Serial.println(lastDuty,3);
    }
  } else if(millis()-lastPrint>=250){
    lastPrint=millis();
    Serial.print(mode==SWINGUP?"swing ":mode==BALANCE?"BAL   ":mode==DECAY?"decay ":"idle  ");
    Serial.print("amp ");    Serial.print(ampDeg,0);
    Serial.print("  half "); Serial.print(halfPeriodMs,0); Serial.print("ms");
    Serial.print("  E ");    Serial.print(E,2); Serial.print("/"); Serial.print(E_BOTTOM,2);
    if(mode==SWINGUP){
      Serial.print("  pumps "); Serial.print(pumps);
      Serial.print(" (+");     Serial.print(lastGain,2); Serial.print(")");
      Serial.print(pumping?"  PUMP":"  centre");
      Serial.print("  x ");    Serial.print(xF*1000,0);
      Serial.print("/");       Serial.print(xExcursion*1000,0); Serial.print("mm");
    }
    if(mode==IDLE && !homed) Serial.print("  [not homed]");
    Serial.print("  duty "); Serial.println(lastDuty,2);
  }
}

/*  WHAT THE STATUS LINE TELLS YOU NOW
 *
 *   swing  amp 118  half 713ms  E -1.02/-3.94  pumps 7 (+0.94)  PUMP  x -88/134mm
 *
 *  amp        must grow. half must stretch with it.
 *  (+n)       what the last reversal bought. Rising = working.
 *  PUMP/centre  which half of the cycle it is in. It should alternate.
 *  x a/b      a = where the cart is now, b = the furthest it has been.
 *
 *  If it stalls, the stop message prints amp, E and the cart's furthest reach.
 *  That tells you which of the three walls you hit:
 *
 *  stalled with cart excursion near 200 mm
 *      Out of rail, which is the 20T pulley limit. Energy per pump scales
 *      with cart SPEED, so a 40T on the motor is the fix: twice the gain per
 *      pump means fewer pumps and less rail used getting there.
 *
 *  stalled with cart excursion well under 150 mm
 *      Not the rail. Run 'd'. Either friction is winning, or the pump is
 *      landing at the wrong moment.
 *
 *  'd' says friction is eating the pump
 *      Mechanical, and no code will fix it. In order of likelihood: the
 *      encoder's own shaft drag, the hub rubbing the bracket face (you want
 *      1 mm of clearance there), the cable tugging the pole, the pole
 *      fouling the limit switch bracket at full swing.
 */
