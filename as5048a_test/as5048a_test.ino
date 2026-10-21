/*  AS5048A elbow sensor test - NO MOTOR.
 *
 *  Reads the AS5048A both ways at once, so either wiring works:
 *    PWM : one signal wire to pin 3 (start here - 3 wires in total)
 *    SPI : CSn -> 10, MOSI -> 11, MISO -> 12, CLK -> 13 (6 wires in total)
 *  and prints 10 times a second, with the pole encoder for comparison:
 *
 *    link1  = pole encoder angle (pins 1/2, as on the rig)
 *    pwm    = AS5048A angle from the PWM pin, and its frame rate
 *    spi    = AS5048A angle over SPI, then AGC and a verdict on the magnet
 *    link2  = link1 + the AS5048A angle (link 2's own angle, once fitted)
 *
 *  POWER IT FROM 3.3 V ONLY. The Teensy's pins are not 5 V tolerant and the
 *  sensor's outputs swing to its supply. If the board has separate 5V and 3V3
 *  pins, connect the Teensy's 3.3 V to BOTH (that is how the chip runs at 3.3 V).
 *
 *  Bench test before the magnet cap exists: hold the magnet flat over the chip,
 *  centred, 1-3 mm away, and turn it. The angle should sweep smoothly through
 *  0-360 and the verdict should say "magnet OK".
 *    "too weak"   -> magnet too far, off centre, or an AXIAL magnet (wrong kind)
 *    "too strong" -> magnet too close
 *    "no pulses" / "no reply" -> that wire is not connected (fine if unused)
 *
 *  z = zero the pole count and the AS5048A angle here
 *
 *  POLE ENCODER PINS ARE FIXED: A = 1, B = 2.
 */

#include <Encoder.h>
#include <SPI.h>

const int PWM_PIN = 6, DIR_PIN = 5;   // motor driver, held off
const int POLE_A = 1, POLE_B = 2;     // FIXED: pole A -> pin 1, pole B -> pin 2
static_assert(POLE_A == 1 && POLE_B == 2, "pole encoder is POLE_A = 1, POLE_B = 2 - do not change");
const int AS_PWM = 3;                 // AS5048A PWM output
const int AS_CS  = 10;                // AS5048A SPI: CSn 10, MOSI 11, MISO 12, CLK 13

const float POLE_DEG_PER_COUNT = 360.0f / 2400.0f;
Encoder poleEnc(POLE_A, POLE_B);

// ---- PWM: time each frame with the 600 MHz cycle counter --------------------
// A frame is 4119 clocks: 12 init + 4 error flag + 4095 data (high for the
// angle count) + 8 exit, so angle counts = high/period * 4119 - 16.
volatile uint32_t riseCyc = 0, highCyc = 0, periodCyc = 0, pulses = 0;
void pwmEdge(){
  uint32_t now = ARM_DWT_CYCCNT;
  if(digitalReadFast(AS_PWM)){ periodCyc = now - riseCyc; riseCyc = now; pulses++; }
  else                       { highCyc = now - riseCyc; }
}
// pwmErr: the frame carried no angle (high for only the 12-clock init part):
// the chip sees no usable magnetic field
bool pwmErr = false;
bool pwmAngle(float &deg, float &hz){
  noInterrupts(); uint32_t h = highCyc, p = periodCyc, n = pulses; interrupts();
  static uint32_t lastN = 0;
  bool fresh = (n != lastN); lastN = n;
  if(!fresh || p == 0) return false;
  float counts = (float)h / (float)p * 4119.0f - 16.0f;
  pwmErr = counts < -2.0f;
  deg = constrain(counts, 0.0f, 4095.0f) * 360.0f / 4096.0f;
  hz  = (float)F_CPU_ACTUAL / (float)p;
  return true;
}

// ---- SPI: 16-bit frames, even parity in bit 15, reply one frame later --------
SPISettings asSpi(1000000, MSBFIRST, SPI_MODE1);
uint16_t withParity(uint16_t w){ return (__builtin_popcount(w & 0x7FFF) & 1) ? (w | 0x8000) : (w & 0x7FFF); }
uint16_t frame(uint16_t out){
  SPI.beginTransaction(asSpi); digitalWriteFast(AS_CS, LOW);
  uint16_t in = SPI.transfer16(out);
  digitalWriteFast(AS_CS, HIGH); SPI.endTransaction();
  delayMicroseconds(1);
  return in;
}
// read a register: returns false if the reply is missing or corrupt
bool spiRead(uint16_t addr, uint16_t &val){
  frame(withParity(0x4000 | addr));
  uint16_t r = frame(0x0000);
  if(r == 0x0000 || r == 0xFFFF) return false;              // nothing on MISO
  if(__builtin_popcount(r) & 1) return false;                // parity
  if(r & 0x4000){ frame(withParity(0x4001)); frame(0x0000); return false; }  // error flag: clear it
  val = r & 0x3FFF; return true;
}

float asZero = 0;
long  poleZero = 0;
uint32_t lastPrint = 0;

float wrap180(float d){ while(d > 180) d -= 360; while(d <= -180) d += 360; return d; }

void setup(){
  pinMode(PWM_PIN, OUTPUT); digitalWrite(PWM_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT); digitalWrite(DIR_PIN, LOW);
  pinMode(AS_PWM, INPUT);
  attachInterrupt(digitalPinToInterrupt(AS_PWM), pwmEdge, CHANGE);
  pinMode(AS_CS, OUTPUT); digitalWriteFast(AS_CS, HIGH);
  SPI.begin();
  Serial.begin(115200);
  uint32_t w = millis(); while(!Serial && millis() - w < 3000){}
  Serial.println("\nAS5048A test. Motor off. Power the sensor from 3.3 V only.");
  Serial.printf("PWM -> pin %d.  SPI: CSn %d, MOSI 11, MISO 12, CLK 13.  Pole encoder A %d, B %d.\n",
                AS_PWM, AS_CS, POLE_A, POLE_B);
  Serial.println("z = zero here");
}

void loop(){
  while(Serial.available()){
    char c = Serial.read();
    if(c == 'z' || c == 'Z'){
      float d, hz; uint16_t a;
      if(spiRead(0x3FFF, a)) asZero = a * 360.0f / 16384.0f;
      else if(pwmAngle(d, hz)) asZero = d;
      poleZero = poleEnc.read();
      Serial.println("# zeroed");
    }
  }
  if(millis() - lastPrint < 100) return;
  lastPrint = millis();

  float link1 = wrap180((poleEnc.read() - poleZero) * POLE_DEG_PER_COUNT);
  Serial.printf("link1 %7.2f |", link1);

  float best = NAN;
  float pd, hz;
  bool got = pwmAngle(pd, hz);
  if(got && pwmErr) Serial.printf(" pwm  NO MAGNET FIELD (%4.0f Hz: sensor alive) - magnet missing, too far, off centre, or axial |", hz);
  else if(got){ Serial.printf(" pwm %7.2f (%4.0f Hz) |", wrap180(pd - asZero), hz); best = pd; }
  else Serial.print(" pwm   no pulses      |");

  uint16_t ang, mag, diag;
  if(spiRead(0x3FFF, ang)){
    float sd = ang * 360.0f / 16384.0f; best = sd;
    Serial.printf(" spi %8.3f", wrap180(sd - asZero));
    if(spiRead(0x3FFE, mag) && spiRead(0x3FFD, diag)){
      int agc = diag & 0xFF;
      bool ocf = diag & 0x100, cof = diag & 0x200, compL = diag & 0x400, compH = diag & 0x800;
      const char *v = !ocf ? "starting up" : cof ? "angle invalid (overflow)" :
                      compL ? "too strong - move the magnet away" :
                      compH ? "too weak - closer, centre it, or the magnet is axial" :
                      (agc < 20) ? "strong but OK" : (agc > 235) ? "weak but OK" : "magnet OK";
      Serial.printf("  agc %3d  mag %5u  %s |", agc, mag, v);
    } else Serial.print(" |");
  } else Serial.print(" spi   no reply |");

  if(!isnan(best)) Serial.printf(" link2 %7.2f", wrap180(link1 + wrap180(best - asZero)));
  Serial.println();
}
