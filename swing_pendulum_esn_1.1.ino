/*  ZIPY cart pole - ECHO STATE NETWORK balance controller, ESN 1.1
 *
 *  ESN 1.1 = ESN 1.0's network on firmware 4.8 (swing_pendulum_claude_1.6).
 *  ESN 1.0 sat on 4.7, whose swing-up could settle part way up - the network
 *  only takes over at the catch, so it never got its turn. 4.8 measures the
 *  pole live on every 'u' and keeps correcting its energy target until the
 *  pole really reaches the top (see the 4.8 notes below).
 *
 *  THE NETWORK (weights in esn_weights.h, made by tools/esn/train_esn.py)
 *    reservoir : 64 leaky-tanh neurons, fixed random recurrent weights,
 *                updated every 2 ms on sin and 1-cos of the angle, angle
 *                rate, cart position, cart speed and the speed command. It
 *                runs in every mode, so its memory is current at the catch.
 *    predict   : linear readout - the pole's TRUE angle and rate 20 ms ahead
 *                (simulated: angle within 0.10 deg; the raw measurement is
 *                1.23 deg off).
 *    control   : linear readout - the cart acceleration, trained by DAgger to
 *                imitate an expert LQR that knows the true state.
 *  Trained on randomised rigs around L = 218 mm (+-8%). If the live
 *  measurement says the pole is far from that, retrain with the new L.
 *
 *  SIMULATED, 40 unseen randomised rigs, 15 s of balancing, held / drops:
 *                                   no shoves   moderate    hard shoves
 *    LQR on measured state          21 / 21     29 / 10     11 / 35
 *    ESN control readout (default)  33 /  0     32 /  2     14 / 35
 *    LQR on ESN-predicted state     32 /  1     31 /  5     13 / 32
 *
 *  KEYS - as 4.8, plus
 *    e  (while idle) cycle the balance controller: ESN -> ESN-predict+LQR -> LQR
 *
 *  ---- the rest of this header is 4.8's ----
 *
 *  ZIPY cart pole - swing up and balance, version 4.8
 *
 *  4.8 ADAPTS IN REAL TIME INSTEAD OF SETTLING HALF WAY. Reported: 1.4 and the
 *  ESN sketch (4.7's swing-up) only got "half way or a bit more" and seemed
 *  "satisfied" there. Simulated: that is exactly what happens when the pole
 *  length in use is too long (e.g. a bad value saved by 'd' and loaded from
 *  EEPROM): 300 mm in use for a 218 mm pole -> stuck at 136 deg for good. The
 *  energy target is met according to the wrong model, and the learnt
 *  correction was capped at 0.076 of the swing energy - far too little.
 *
 *  1. LIVE POLE MEASUREMENT ON EVERY 'u'. Pump to a ~35 deg swing, stop the
 *     cart, time three free half swings (~1.5 s), set L, carry on. Saved to
 *     EEPROM if it changed. Simulated: 218 mm to within 0.3% starting from
 *     150, 218, 300 or 402 mm, then caught and held ~40 s.
 *  2. THE LEARNT ENERGY TARGET MAY MOVE UP TO 0.6 OF THE SWING ENERGY (was
 *     0.076), so even a measurement that is off keeps being corrected until
 *     the pole really reaches the top.
 *  Simulated, 32 rig variants (motor 30-90 ms, friction incl. stiction, pole
 *  friction, start off centre), firmware STARTING from a wrong 300 mm:
 *    4.7: 0/32 balanced (stuck part way)   4.8: 30/32 balanced
 *  (4.7 started from the right 218 mm managed 29/32.)
 *  A "only hand over near the top" guard was also tried: it gets the pole up
 *  whatever the length, but cost 11/32 at the balance - rejected.
 *  The status line shows MEASURE while the cart is stopped, and the learnt
 *  target (fraction of swing energy) during the final approach.
 *
 *  ---- 4.7 ----
 *
 *  4.7 = 4.6 WITH YOUR POLE'S MEASURED LENGTH BUILT IN. Both 2026-10-15 logs
 *  that "still only got half way" were run on 1.3 (banner "v4.4", old 'c'
 *  output, 'u' without 'd') - 4.6 was never loaded. One of them contains a
 *  free-decay test, and its swing periods (876/855/838/832 ms full period at
 *  80-65 deg) put the pole at 218 mm: a 402 mm rod would take 1.04 s, this one
 *  takes 0.765 s. That is now the default, so 'u' works without 'd'.
 *  Simulated on the rig's numbers (218 mm, V 0.317, tau ~60 ms, creep 0.18):
 *    1.2 logic: reaches the top in 5 s, then 72+ catch/drop cycles, holds 1-2 s
 *    1.3 logic: stalls at ~108 deg - exactly the two logs
 *    4.6/4.7  : reaches the top, ONE catch, holds 30+ s
 *  Also: the near-the-top time limit is 45 s (was 30).
 *  CHECK THE BOOT BANNER SAYS v4.7 before trusting a run.
 *
 *  ---- 4.6 ----
 *
 *  4.6 MEASURES THE POLE INSTEAD OF ASSUMING IT. Seen in the 2026-10-15 log:
 *  the swing stalled at ~105 deg from hanging, the energy reading jumped by
 *  ~2 J/kg within a single swing, and the (+n) gains alternated +1.5/-1.7.
 *  Every formula assumed a 402 mm uniform rod, but the pole swings like a
 *  ~225 mm one (period ~0.96 s at that height, against ~1.3 s for 402 mm).
 *  The simulator reproduces the stall exactly with a 0.20-0.25 m pole and the
 *  firmware still assuming 0.402. With the energy reading wrong, the smooth
 *  near-the-top control took over far too early and could not add the energy
 *  still missing. The swing period alone fixes the model: for any rigid pole
 *  it behaves exactly like a uniform rod of L = 3*g*T0^2 / (8*pi^2).
 *
 *  1. 'd' NOW MEASURES THE POLE. It times the free swings (corrected for
 *     amplitude), works out L, uses it at once and SAVES it in EEPROM, so it
 *     survives power-off. Simulated: within 0.1% from a 30-90 deg release,
 *     light or heavy friction. 'u' refuses until the pole has been measured.
 *  2. EVERY ENERGY THRESHOLD SCALES WITH L (they were fixed numbers worked out
 *     for 402 mm - e.g. the bottom is -2.2 for 225 mm, not -3.94), and the
 *     balance gains are re-derived for L from a table.
 *  3. THE ENERGY TARGET IS LEARNT FROM WHAT HAPPENS AT THE TOP. Fell back
 *     short: raise the target by the shortfall. Went over: lower it by the
 *     excess. 4.5 aimed slightly over the top from the energy reading alone;
 *     with a 3% length error it missed the catch and held the pole in a slow
 *     endless rotation. Simulated with learning: balances with the length
 *     anywhere from -12% to +6% off (without: only when exact).
 *  4. 'c' READ THE POLE SIGN TOO LATE. It looked 120 ms after the cart had
 *     stopped, by which time the pole had swung back the other way - so it
 *     got the sign wrong and swing-up wasted 12 s finding it. Now it takes
 *     the pole's FIRST move while the cart is speeding up.
 *  5. A BOTTOM CROSSING LANDING EXACTLY ON ZERO WAS MISSED. phi*phiPrev < 0 is
 *     false when a sample is exactly 0 (encoder count exactly 0), so the decay
 *     test dropped whole swings. Sign comparisons now count zero as positive.
 *  6. CATCH WINDOW 20 -> 15 DEG. A shorter pole falls faster, and catches at
 *     the 20 deg edge ran the cart out of speed.
 *
 *  THE SIGN TRAP line in the swing-up is untouched. Pole pins stay 1 and 2.
 *
 *  4.5 FIXES THE BALANCE AND THE ENDLESS CATCH / DROP / SPIN LOOP.
 *  Seen in the 2026-10-14 logs: every catch was at the very edge of the old
 *  window (34-35 deg) and moving fast; after a drop the swing-up kept pumping
 *  energy into a pole that already had plenty, until it spun in full circles
 *  at ~1000 deg/s; the spinning pole kept passing through the window and got
 *  "caught" again; and the retry counter was reset on every catch, so the
 *  5-try limit never came. One catch did hold ~1.7 s - and the cart crept
 *  steadily to the fence until it ran out of room.
 *
 *  All of this was reproduced in a simulation of the rig, and every change
 *  below was tested there (fast, medium and slow motor; light to heavy and
 *  lopsided friction; starts off centre; a spinning pole; shoves mid-balance).
 *
 *  1. THE BALANCE DRIVES SPEED, NOT ACCELERATION. The motor is a speed source:
 *     duty sets how fast the cart runs, almost at once. 4.4 fed the balance
 *     controller's acceleration demand straight in as duty, so the cart crept
 *     off to the fence (simulated: falls in ~2 s, exactly as logged). Now the
 *     demand is integrated into a speed command, then turned into duty.
 *     Simulated: holds indefinitely, cart within a few mm of centre. The LQR
 *     gains themselves were re-derived and are right.
 *  2. SMOOTH ENERGY CONTROL NEAR THE TOP. The full-power pumping stays for the
 *     climb, but above E = -0.8 it hands over to a gentle continuous push that
 *     steers the energy to just-reaches-the-top: adds when short, BRAKES when
 *     there is too much. That is what stops the spinning. A slow correction
 *     raises the target if friction keeps it falling short.
 *  3. CATCH ONLY WHAT CAN BE HELD. The cart can only hold the pole within ~13
 *     deg of upright, so a catch at 35 deg moving fast was lost before it
 *     started. Now: within 20 deg AND on course to arrive at the top neither
 *     too fast nor too slow (the "z" test - see catchable()).
 *  4. DROPS ARE COUNTED PROPERLY. A drop counts until the pole has been held
 *     for 3 s. Five drops in a row and it stops. Recovering from a drop no
 *     longer fires the 0.2 s start-up kick, and the automatic sign flip is
 *     switched off once a catch has proved the sign - near the top the energy
 *     is held, not climbing, and 4.4's check could flip a correct sign there.
 *  5. SAFETY. 'u' and 'b' need 'h' first (the fence is measured, not
 *     guessed), and the pole spinning faster than ~E = 6 stops everything.
 *  6. The status line shows the angle, cart and hold time while balancing,
 *     and every catch prints the numbers it was judged on.
 *
 *  THE SIGN TRAP line in the swing-up is untouched.
 *
 *  4.4 MAKES HOMING COPE WITH THE RIGHT-HAND END. Seen on the rig: 'h' went
 *  left, came back right, then stayed at the right and never centred. Moving
 *  right is the high-friction direction - the tight belt span wraps the
 *  encoder pulley at that end - so 0.22 duty can stall part way, which homing
 *  took for the end of the rail and then rejected as a short measurement.
 *  Or the cart wedges at that end and 0.26 duty cannot pull it free.
 *
 *  1. A stall during homing gets one harder push (+0.08, up to 0.38). If the
 *     cart moves, it was friction and the run carries on; if not, it is the end.
 *  2. Each run prints how far it went and at what duty, and a failed homing
 *     says whether the encoder saw nothing at all (wiring) or the cart stopped
 *     short (something blocking it).
 *  3. Centring steps the duty up (to 0.50) while the cart is not moving, and
 *     drops back once it is free.
 *  4. The idle status line shows the cart position and the pole angle, so you
 *     can see the encoders counting without loading encoder_test.
 *
 *  Swing-up and balance are untouched.
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
#include <EEPROM.h>
#include "esn_weights.h"

const int PWM_PIN = 6, DIR_PIN = 5;
const int POLE_A = 1, POLE_B = 2;     // FIXED: pole A -> pin 1, pole B -> pin 2
const int CART_A = 8, CART_B = 9;
static_assert(POLE_A == 1 && POLE_B == 2, "pole encoder is POLE_A = 1, POLE_B = 2 - do not change");

const float M_PER_COUNT   = 0.040f / 2400.0f;
const float RAD_PER_COUNT = 2.0f * PI / 2400.0f;

// L is the length of the uniform rod that swings like YOUR pole. Default: 218
// mm, MEASURED from the free-swing periods in logs/20261015-002708.txt (full
// period 876/855/838/832 ms at 80-65 deg -> small-swing T0 0.765 s -> 218 mm).
// The old 402 mm was wrong by ~85%. 'd' re-measures it and saves to EEPROM.
const float G = 9.81f;
const float L_GUESS = 0.218f;
float L = L_GUESS;
bool  poleMeasured = false;
float E_BOTTOM = -G * L_GUESS;     // these three follow L - see setPoleLength()
float LAMBDA   = sqrtf(1.5f * G / L_GUESS);   // how fast it falls from upright, 1/s
float B_POLE   = 1.5f / L_GUESS;              // pole reaction to cart acceleration
const float D2R = PI / 180.0f;

float SIGN_X = +1.0f, SIGN_THETA = +1.0f;
float A_MAX = 2.2f, V_MAX = 0.367f, FRICTION_FF = 0.08f;

// ---- swing-up ------------------------------------------------------------
float DUTY_SWING = 0.95f;
const float PHI_DEAD  = 0.035f;   // hold direction within 2 deg of the bottom
const float SENS_GATE = 0.35f;    // pump while |wF*cos th| is above this
                                  // fraction of its recent peak; recentre below
float KX_R = 3.2f, KV_R = 1.1f;   // recentring gains, in DUTY units

// ---- energy thresholds, as a FRACTION of the full swing energy G*L ---------
// En = E/(G*L): -1 hanging still, 0 just reaching the top. 4.5 used fixed J/kg
// numbers worked out for 402 mm; these are the same values divided by 3.944.
const float EN_CONSERVE = -0.811f; // below: pump flat out (was E -3.2)
const float EN_SWITCH   = -0.203f; // above: smooth energy control (was -0.8)
const float EN_SW_BACK  = -0.406f; // fall below this to go back to pumping
const float EN_SIGNCHK  = -0.507f; // sign check only below this (was -2.0)
const float EN_NEAR     =  0.076f; // within this of the target = progress (was 0.3)
const float EN_BIAS_MAX =  0.6f;   // learnt target offset limit. 4.7 had 0.076: too
                                   // small to correct a big model error, so with a wrong
                                   // length the pole settled for good at 136-159 deg
const float EN_SPIN     =  1.52f;  // above: spinning, stop (was 6.0)

// ---- final approach: smooth energy control (4.5, target learnt in 4.6) ----
const float KE_N   = 15.8f;        // push (m/s^2) per unit of En error (was 4 per J/kg)
const float LEARN  = 0.5f;         // fraction of each miss corrected next time
const float KXE = 6.0f, KVE = 4.0f;  // meanwhile, ease the cart back to the middle
const float A_LIM  = 6.0f;         // m/s^2

// ---- catch: only a pole the balance can actually hold (4.5) ---------------
const float TH_CATCH = 15.0f;     // deg from upright. 4.5 had 20: for a short pole
                                  // (it falls faster) catches at the 20 deg edge ran
                                  // the cart out of speed. Simulated 15 vs 20: 48/48
                                  // vs 40/48 re-held after shoves, 17/18 vs 13/18 rigs
const float ZO_CATCH = 0.09f;     // z window: arriving too fast (would go over)
const float ZU_CATCH = 0.06f;     // z window: arriving too slow (would fall back)

// ---- balance -------------------------------------------------------------
float KX = 3.46f, KV = 4.38f, KTH = 35.58f, KW = 5.40f;   // LQR, acceleration out - set by setPoleLength()
const float GIVEUP_DEG = 30.0f;   // past this it cannot be recovered - let it go
const float DUTY_MAX   = 0.98f;
const float BAL_FF     = 0.08f;   // fixed friction step for speed commands. NOT the
                                  // measured creep duty: friction here is lopsided,
                                  // and one big number both ways over-corrects
const uint32_t HOLD_MS = 3000;    // held this long = a real catch, drops reset
const int   MAX_DROPS  = 5;       // dropped this many times in a row = stop

// LQR gains (Q = diag(12,1,140,12), R = 1, acceleration input) against L,
// solved with scipy in tools/sim (lqr_gains). Linear between rows - checked:
// the error at the midpoints is under 0.01%.
const float K_TABLE[][5] = {
  // L      KX      KV      KTH      KW
  {0.150f, 3.464f, 4.303f, 34.615f, 4.774f},
  {0.175f, 3.464f, 4.330f, 34.940f, 4.985f},
  {0.200f, 3.464f, 4.356f, 35.260f, 5.196f},
  {0.225f, 3.464f, 4.381f, 35.576f, 5.405f},
  {0.250f, 3.464f, 4.406f, 35.888f, 5.613f},
  {0.275f, 3.464f, 4.431f, 36.196f, 5.820f},
  {0.300f, 3.464f, 4.455f, 36.500f, 6.026f},
  {0.325f, 3.464f, 4.479f, 36.801f, 6.231f},
  {0.350f, 3.464f, 4.503f, 37.100f, 6.435f},
  {0.375f, 3.464f, 4.526f, 37.395f, 6.638f},
  {0.400f, 3.464f, 4.548f, 37.687f, 6.841f},
  {0.425f, 3.464f, 4.571f, 37.976f, 7.043f},
  {0.450f, 3.464f, 4.593f, 38.263f, 7.245f},
};
const int K_ROWS = sizeof(K_TABLE) / sizeof(K_TABLE[0]);
const float L_MIN = 0.12f, L_MAX = 0.50f;      // a measurement outside this is rejected

// EEPROM: where the measured pole length is kept between power-ups
struct PoleCal { uint32_t magic; float L; };
const uint32_t POLE_MAGIC = 0x5A495059;        // "ZIPY"

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
const float HOME_DUTY     = 0.22f; // homing creep
const float HOME_DUTY_MAX = 0.38f; // harder push to tell friction from the end
const float CENTRE_DUTY_MAX = 0.50f;
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

int dirHold=+1, pumps=0, flips=0, drops=0;
float vCmd=0, eBias=0;             // speed command (m/s), learnt energy target (J/kg)
float thLast=0;                    // last tick's angle, for spotting a crossing of upright
bool  ecOn=false, signProven=false, held=false;
uint32_t balStartMs=0, ecSinceMs=0;
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
uint32_t decayTms[13];             // when each turning point happened
float    decayAmp[13];             // and how far out it was (rad from hanging)
uint32_t decayStartAt=0, decayTurnAt=0;

float wrapPi(float a){ while(a>PI)a-=2*PI; while(a<-PI)a+=2*PI; return a; }
float thetaUp(){ return SIGN_THETA * wrapPi(poleEnc.read()*RAD_PER_COUNT - PI); }
float cartX()  { return SIGN_X * cartEnc.read() * M_PER_COUNT; }
float energy(float th,float w){ return (L*L/6.0f)*w*w + (G*L/2.0f)*(cosf(th)-1.0f); }

// Everything that depends on the pole follows L from here: the energy scale,
// how fast it falls, its reaction to the cart, and the balance gains.
void setPoleLength(float Lnew){
  L = Lnew;
  E_BOTTOM = -G * L;
  LAMBDA   = sqrtf(1.5f * G / L);
  B_POLE   = 1.5f / L;
  float Lc = constrain(L, K_TABLE[0][0], K_TABLE[K_ROWS-1][0]);
  int i = 0;
  while(i < K_ROWS-2 && Lc > K_TABLE[i+1][0]) i++;
  float f = (Lc - K_TABLE[i][0]) / (K_TABLE[i+1][0] - K_TABLE[i][0]);
  KX  = K_TABLE[i][1] + f*(K_TABLE[i+1][1] - K_TABLE[i][1]);
  KV  = K_TABLE[i][2] + f*(K_TABLE[i+1][2] - K_TABLE[i][2]);
  KTH = K_TABLE[i][3] + f*(K_TABLE[i+1][3] - K_TABLE[i][3]);
  KW  = K_TABLE[i][4] + f*(K_TABLE[i+1][4] - K_TABLE[i][4]);
}
void loadPoleLength(){
  PoleCal c; EEPROM.get(0, c);
  if(c.magic == POLE_MAGIC && c.L >= L_MIN && c.L <= L_MAX){ setPoleLength(c.L); poleMeasured = true; }
  else { setPoleLength(L_GUESS); poleMeasured = false; }
}
void savePoleLength(){
  PoleCal c = { POLE_MAGIC, L };
  EEPROM.put(0, c);
}

// Small-swing period from one measured at amplitude A (rad): a pendulum slows
// down as it swings wider, T(A) = T0 / AGM(1, cos(A/2)). Exact, not a series.
float smallSwingPeriod(float T, float A){
  float a = 1.0f, b = cosf(A * 0.5f);
  for(int i=0; i<6; i++){ float an = 0.5f*(a+b); b = sqrtf(a*b); a = an; }
  return T * a;
}

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

// The motor is a SPEED source: duty sets how fast the cart runs, within a few
// ms. So anything that wants an acceleration integrates it into vCmd (m/s)
// and drives vCmd through here. V_MAX is the speed at 0.95 duty ('m' measures
// it); BAL_FF is the step needed to get moving, smoothed near zero speed.
void addAccel(float a, float dt){
  const float vlim = 0.95f * V_MAX;
  vCmd += a*dt;
  if(vCmd >  vlim) vCmd =  vlim;
  if(vCmd < -vlim) vCmd = -vlim;
}
float speedToDuty(float v){
  float ff = v / 0.02f; if(ff > 1) ff = 1; if(ff < -1) ff = -1;
  return v * (0.95f - BAL_FF) / V_MAX + BAL_FF * ff;
}
// After the fence trims a duty, pull vCmd back to match it - otherwise vCmd
// keeps growing against the fence and lurches out when the fence lets go.
float dutyToSpeed(float d){
  if(fabsf(d) <= BAL_FF) return 0.0f;
  return (d - copysignf(BAL_FF, d)) * V_MAX / (0.95f - BAL_FF);
}

// Can the balance controller hold the pole from here? Linearised near the top
// the pole runs away like z = th + w/LAMBDA + (B_POLE/LAMBDA)*v: z = 0 is the
// path that coasts exactly to upright and stops; z on the wrong side means it
// will go over or fall back. With the cart's limited speed only a narrow band
// of z can be rescued - simulated, overshoot to -0.12, fall-back to +0.06 for
// a medium motor, narrower for a slow one - so the window sits inside that.
float catchZ(){
  float z = thF + wF/LAMBDA + (B_POLE/LAMBDA)*vF;
  return (thF > 0) ? z : -z;           // + = arriving too slow, - = too fast
}
bool catchable(){
  float zn = catchZ();
  return fabsf(thF)/D2R < TH_CATCH && zn > -ZO_CATCH && zn < ZU_CATCH;
}

void drive(float d){
  if(d> DUTY_MAX) d= DUTY_MAX;
  if(d<-DUTY_MAX) d=-DUTY_MAX;
  lastDuty=d;
  digitalWrite(DIR_PIN, d>=0?HIGH:LOW);
  analogWrite(PWM_PIN,(int)(fabsf(d)*4095));
}
void halt(const char*w){ drive(0); mode=IDLE; Serial.print("-- stopped: "); Serial.println(w); }

// ---- echo state network ----------------------------------------------------
float esnR[ESN_N];                 // reservoir state
float esnU[ESN_U];                 // inputs this tick
float esnPhi[ESN_F];               // features for the readouts: [reservoir, inputs, 1]
enum BalCtl { CTL_ESN, CTL_ESN_PRED, CTL_LQR };
BalCtl balCtl = CTL_ESN;
const char* balCtlName(){
  return balCtl==CTL_ESN ? "ESN" : balCtl==CTL_ESN_PRED ? "ESN-predict+LQR" : "LQR";
}

// One reservoir update, exactly as tools/esn/train_esn.py does it:
// r <- (1-leak)*r + leak*tanh(Win*u + W*r + b), using the OLD r on the right.
void esnStep(){
  esnU[0] = sinf(thF)        / ESN_IN_NORM[0];
  esnU[1] = (1.0f-cosf(thF)) / ESN_IN_NORM[1];
  esnU[2] = wF               / ESN_IN_NORM[2];
  esnU[3] = xF               / ESN_IN_NORM[3];
  esnU[4] = vF               / ESN_IN_NORM[4];
  esnU[5] = vCmd             / ESN_IN_NORM[5];
  static float pre[ESN_N];
  for(int i=0; i<ESN_N; i++){
    float s = ESN_B[i];
    const float* w  = &ESN_W[i*ESN_N];
    for(int j=0; j<ESN_N; j++) s += w[j]*esnR[j];
    const float* wi = &ESN_WIN[i*ESN_U];
    for(int j=0; j<ESN_U; j++) s += wi[j]*esnU[j];
    pre[i] = s;
  }
  for(int i=0; i<ESN_N; i++) esnR[i] = (1.0f-ESN_LEAK)*esnR[i] + ESN_LEAK*tanhf(pre[i]);
  for(int i=0; i<ESN_N; i++) esnPhi[i] = esnR[i];
  for(int j=0; j<ESN_U; j++) esnPhi[ESN_N+j] = esnU[j];
  esnPhi[ESN_F-1] = 1.0f;
}
float esnDot(const float* w){
  float s = 0;
  for(int i=0; i<ESN_F; i++) s += w[i]*esnPhi[i];
  return s;
}
float esnPredAngle(){ return thF + esnDot(&ESN_WP[0]); }      // true angle 20 ms ahead
float esnPredRate() { return wF  + esnDot(&ESN_WP[ESN_F]); }  // true rate 20 ms ahead

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
// If the cart is not moving - wedged at an end, or friction - the duty steps
// up, and drops back to the base once it is free so it does not overshoot.
bool centreCart(uint32_t timeoutMs){
  const float base = fmaxf(0.26f, FRICTION_FF + 0.06f);
  float cd = base;
  uint32_t t0c=millis(), movedAt=millis();
  long lastC = cartEnc.read();
  while(fabsf(cartX()) > 0.004f && millis()-t0c < timeoutMs){
    drive(cartX() > 0 ? -cd : cd);
    if(waitMs(10)) return true;
    long c = cartEnc.read();
    if(labs(c-lastC) > 3){ lastC=c; movedAt=millis(); cd=base; }
    else if(millis()-movedAt > 300 && cd < CENTRE_DUTY_MAX){
      cd = fminf(cd + 0.04f, CENTRE_DUTY_MAX); movedAt=millis();
      Serial.print("# cart not moving - centring duty up to "); Serial.println(cd,2);
    }
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
  Serial.print("  FRIC ");Serial.print(FRICTION_FF,3);
  Serial.print("  POLE ");Serial.print(L*1000,0);
  Serial.println(poleMeasured ? " mm (measured by d)" : " mm (from the 10-15 decay log)");
}

// ---- live pole measurement (4.8) -----------------------------------------
// At the start of every 'u': pump the pole to a ~35 deg swing, stop the cart,
// time three free half swings and set L from them - the same sums as 'd'.
// A wrongly remembered length (EEPROM) or a changed pole can then no longer
// leave the swing-up "satisfied" part way up. Simulated: measures 218 mm to
// within 0.3% whether it starts from 150, 218, 300 or 402 mm, then catches.
enum CalPhase { CAL_PUMP, CAL_FREE, CAL_DONE };
CalPhase calPhase = CAL_DONE;
bool     calCrossed = false;
float    calPeak = 0, calPhiPrev = 0;
int      calSkip = 0, calN = 0;
uint32_t calT[4], calFreeSince = 0;
float    calA[4];
const float CAL_AMP_DEG = 35.0f;  // swing to reach before stopping the cart
const int   CAL_TURNS   = 4;      // turning points to time (3 half periods)

void finishLiveCal(){
  calPhase = CAL_DONE;
  if(calN < 2){
    Serial.println("!! live pole measurement: too few free swings - keeping the old length");
  } else {
    float Ls = 0; int n = 0;
    for(int i=1; i<calN; i++){
      float T0 = smallSwingPeriod(2.0f*(calT[i]-calT[i-1])/1000.0f, 0.5f*(calA[i]+calA[i-1]));
      Ls += 3.0f*G*T0*T0 / (8.0f*PI*PI); n++;
    }
    float Lm = Ls / n;
    if(Lm >= L_MIN && Lm <= L_MAX){
      float Lold = L;
      setPoleLength(Lm); poleMeasured = true;
      if(fabsf(Lm - Lold) > 0.005f) savePoleLength();
      Serial.print("# pole measured live: "); Serial.print(L*1000,0);
      Serial.print(" mm (was "); Serial.print(Lold*1000,0); Serial.println(" mm). Carrying on.");
    } else {
      Serial.print("!! live pole measurement came out "); Serial.print(Lm*1000,0);
      Serial.println(" mm - not believable, kept the old length");
    }
  }
  // the free swings were not a stall - restart the progress clocks
  bestAt = millis(); signCheckAt = millis()+3000;
  eAtCheck = energy(thetaUp(),0); eBest = eAtCheck;
}

// Runs every swing-up tick until the measurement is done. Returns true while
// the cart is stopped and the pole swinging freely (nothing else to do then).
bool calFreeTick(float phi){
  if(calPhase == CAL_DONE) return false;
  // turning points, exactly as the 'd' test finds them (zero-safe crossing)
  if((phi >= 0) != (calPhiPrev >= 0) && fabsf(phi) < 1.0f){ calCrossed = true; calPeak = 0; }
  calPhiPrev = phi;
  if(fabsf(phi) > calPeak) calPeak = fabsf(phi);
  bool turned = (wF >= 0) != (wPrevSign >= 0);
  if(turned && calCrossed && calPeak > 0.12f){
    calCrossed = false;
    if(calPhase == CAL_PUMP){
      if(calPeak/D2R >= CAL_AMP_DEG){
        calPhase = CAL_FREE; calSkip = 1; calN = 0; calFreeSince = millis();
        Serial.println("# measuring the pole live: cart stopped ~1.5 s, pole swinging freely");
      }
    } else if(calSkip > 0) calSkip--;               // first turn: the cart was still stopping
    else if(calN < CAL_TURNS){ calT[calN] = millis(); calA[calN] = calPeak; calN++; }
    calPeak = 0;
  }
  if(calPhase != CAL_FREE) return false;
  if(calN >= CAL_TURNS || millis() - calFreeSince > 4000){
    finishLiveCal();
    return false;                                     // carry on swinging up this tick
  }
  drive(0); wPrevSign = wF;
  return true;
}

// fresh = started by 'u' with the pole hanging. Not fresh = recovering after a
// drop: no start-up kick (the pole is already moving - 4.4 kicked it anyway),
// and the drop count, energy correction and proven sign are kept.
void startSwing(bool fresh){
  mode=SWINGUP;
  dirHold=+1; pumps=0;
  kickUntil = fresh ? millis()+200 : millis();
  signCheckAt=millis()+3000;
  eAtCheck=energy(thetaUp(),0); eBest=eAtCheck; bestAt=millis();
  eAtPump=eAtCheck; lastGain=0; sensPk=0.5f; xExcursion=0;
  ecOn=false;
  if(fresh){
    flips=0; drops=0; eBias=0; signProven=false;
    calPhase=CAL_PUMP; calCrossed=false; calPeak=0; calPhiPrev=0; calN=0;   // measure the pole on the way
    Serial.println("# pumping: drive away from the pole near the bottom, recentre near the ends.");
  }
}

void calibrate(){
  Serial.println("# calibrating - cart moves ~40 mm. Pole hanging still. x stops it.");
  // The cart is measured in relative counts: zeroing it here threw away the
  // origin that 'h' had measured. The pole IS zeroed - it is hanging still.
  poleEnc.write(0);
  if(waitMs(300)){ halt("by hand, during calibrate"); return; }
  long c0=cartEnc.read(), p0=poleEnc.read();
  drive(0.55f);
  // The pole's FIRST move is the one that tells the sign: while the cart speeds
  // up, the pole lags behind it. 4.5 looked 120 ms after the stop - by then the
  // sharp stop had swung the pole the other way, and on this rig it got the
  // sign wrong every time (swing-up then wasted 12 s finding it).
  long firstDp = 0;
  bool stopped = false;
  uint32_t tp = millis();
  while(millis()-tp < 220){
    if(stopPressed()){ stopped = true; break; }
    long d = poleEnc.read() - p0;
    if(firstDp == 0 && labs(d) >= 3) firstDp = d;     // 3 counts = 0.45 deg
  }
  drive(0);
  long dx=cartEnc.read()-c0;
  if(stopped || waitMs(120)){ halt("by hand, during calibrate"); return; }
  long dp=poleEnc.read()-p0;
  if(waitMs(900)){ halt("by hand, during calibrate"); return; }
  if(dx==0){ Serial.println("!! cart encoder did not move. Belt slipping, or wiring."); mode=IDLE; return; }
  if(firstDp==0) Serial.println("!! pole barely moved while the cart sped up - set screw? Sign taken from the end instead.");
  long signDp = firstDp ? firstDp : dp;
  SIGN_X     = (dx>0)? +1.0f : -1.0f;
  SIGN_THETA = (signDp<0)? -1.0f : +1.0f;
  Serial.print("# cart "); Serial.print(dx*M_PER_COUNT*1000,1);
  Serial.print(" mm, pole first moved "); Serial.print(firstDp*RAD_PER_COUNT/D2R,2);
  Serial.print(" deg (sign from this), ended at "); Serial.print(dp*RAD_PER_COUNT/D2R,1); Serial.println(" deg");
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
  const char* side[2] = { "-duty end (left?)", "+duty end (right?)" };
  long ends[2];
  float legMm[2];
  for(int i=0;i<2;i++){
    const float sgn = (i==0) ? -1.0f : +1.0f;
    float hd = HOME_DUTY;
    long legStart = cartEnc.read();
    drive(sgn*hd);
    long last = legStart;
    uint32_t still = millis(), t0h = millis();
    while(millis()-t0h < 20000){
      if(waitMs(20)){ halt("by hand, during homing - fence unchanged"); return; }
      long c = cartEnc.read();
      if(labs(c-last) > 3){ last = c; still = millis(); }
      else if(millis()-still > 250){
        // Stalled: the end of the rail, or just friction? Push harder once.
        // If it moves, it was friction - carry on at the higher duty. It has
        // to travel a real 1 mm: pressed harder against the end, the printed
        // parts flex a few counts, and that is not the cart breaking free.
        if(hd < HOME_DUTY_MAX - 0.001f){
          hd = fminf(hd + 0.08f, HOME_DUTY_MAX);
          drive(sgn*hd);
          if(waitMs(250)){ halt("by hand, during homing - fence unchanged"); return; }
          long c2 = cartEnc.read();
          if(labs(c2-last) * M_PER_COUNT > 0.001f){
            Serial.print("# stalled part way to the "); Serial.print(side[i]);
            Serial.print(" - friction. Duty up to "); Serial.println(hd,2);
            last = c2; still = millis();
            continue;
          }
        }
        break;                                      // will not move = at the end
      }
    }
    drive(0);
    if(waitMs(450)){ halt("by hand, during homing - fence unchanged"); return; }
    ends[i] = cartEnc.read();
    legMm[i] = labs(ends[i]-legStart) * M_PER_COUNT * 1000.0f;
    Serial.print("# "); Serial.print(side[i]); Serial.print(": moved ");
    Serial.print(legMm[i],0); Serial.print(" mm, last duty "); Serial.println(hd,2);
  }
  long  span = ends[1]-ends[0];       // raw counts, -duty end to +duty end
  long  mid  = (ends[0]+ends[1])/2;
  float half = fabsf(span * M_PER_COUNT) / 2.0f;
  if(half < 0.10f || half > 0.45f){
    Serial.print("!! measured half travel "); Serial.print(half*1000,0);
    Serial.println(" mm - expected about 230. Fence left at its default.");
    if(legMm[0] < 5.0f && legMm[1] < 5.0f){
      Serial.println("   The cart encoder saw NO movement either way.");
      Serial.println("   If the cart DID move: the cart encoder is not counting - check");
      Serial.println("   pins 8/9, red on VIN, black on GND, or run encoder_test.");
      Serial.println("   If it did NOT move: the motor is not getting drive (run motor_test).");
    } else {
      Serial.println("   The cart stopped short. Something is blocking it, the belt slipped,");
      Serial.println("   or it needs more than 0.38 duty in that direction.");
    }
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
  if(centreCart(12000)){ halt("by hand, while centring - homing result kept"); return; }
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
  Serial.println("# It times the swings to MEASURE THE POLE (saved for next time), and tells");
  Serial.println("# you if friction is beating you.");
}

// Called after 13 turns, or earlier if the swing dies out first - 4.2 only
// ever finished at 13, so a pole that stopped sooner left it in DECAY for ever.
void finishDecay(){
  int n = decayN-1;                       // half swings between the turns

  // --- the pole's length, from how long each half swing took
  float Lsum = 0; int Ln = 0;
  for(int i=1; i<decayN; i++){
    float Th = (decayTms[i] - decayTms[i-1]) / 1000.0f;            // half period, s
    float T0 = smallSwingPeriod(2.0f*Th, 0.5f*(decayAmp[i] + decayAmp[i-1]));
    Lsum += 3.0f*G*T0*T0 / (8.0f*PI*PI); Ln++;
  }
  float Lmeas = Lsum / Ln;
  Serial.println();
  if(Lmeas >= L_MIN && Lmeas <= L_MAX){
    float Lold = L;
    setPoleLength(Lmeas); poleMeasured = true; savePoleLength();
    Serial.print("# POLE: swings like a uniform rod of "); Serial.print(L*1000,0);
    Serial.print(" mm (was "); Serial.print(Lold*1000,0); Serial.println(" mm). Saved - no need to repeat.");
    Serial.print("#   balance gains now KX "); Serial.print(KX,2); Serial.print(" KV "); Serial.print(KV,2);
    Serial.print(" KTH "); Serial.print(KTH,2); Serial.print(" KW "); Serial.println(KW,2);
  } else {
    Serial.print("!! pole length came out "); Serial.print(Lmeas*1000,0);
    Serial.println(" mm - not believable. Kept the old value. Was the cart held still?");
  }

  // --- friction, in energy terms (recomputed with the measured length)
  float eFirst = (G*L/2.0f)*(1.0f - cosf(decayAmp[0]));
  float eLast  = (G*L/2.0f)*(1.0f - cosf(decayAmp[decayN-1]));
  decayLoss = (eFirst - eLast)/n;
  float gain = pumpGain(decayBigA);
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
    // 'u' and 'b' need homing: balancing runs the cart close to the fence, and
    // an unhomed fence is measured from wherever the cart sat at power-up.
    case 'u': case 'U':
      if(!homed){ Serial.println("!! not homed - press h first (the fence must be measured, not guessed)"); break; }
      startSwing(true); break;
    case 'b': case 'B':
      if(!homed){ Serial.println("!! not homed - press h first (the fence must be measured, not guessed)"); break; }
      mode=BALANCE; drops=0; vCmd=vF; balStartMs=millis(); held=false;
      Serial.println("# balancing - hold the pole upright, then let go"); break;
    case 'p': case 'P': printNums(); break;
    case 'e': case 'E':
      balCtl = (balCtl==CTL_ESN) ? CTL_ESN_PRED : (balCtl==CTL_ESN_PRED) ? CTL_LQR : CTL_ESN;
      Serial.print("# balance controller: "); Serial.println(balCtlName()); break;
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
  Serial.println("\nZIPY ESN 1.1 (swing-up 4.8 + echo state network balance). Motor off.");
  Serial.print("Balance controller: "); Serial.print(balCtlName());
  Serial.print("  (e cycles ESN / ESN-predict+LQR / LQR)  reservoir "); Serial.print(ESN_N); Serial.println(" neurons");
  Serial.print("Pole encoder A -> pin "); Serial.print(POLE_A);
  Serial.print(", B -> pin "); Serial.println(POLE_B);
  loadPoleLength();
  Serial.print("Pole length "); Serial.print(L*1000,0);
  Serial.println(poleMeasured ? " mm (measured by d, from EEPROM)." : " mm (from the 10-15 decay log; d refines it).");
  Serial.println("Order:  z  ->  c  ->  h  ->  m  ->  u   (d optional: re-measures the pole)");
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
  esnStep();                       // every tick, every mode: keep the network's memory current
  if((mode==SWINGUP || mode==BALANCE) && E > EN_SPIN*G*L)
    halt("pole spinning too fast - stopped for safety");
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
    // Sign compare, zero counted as positive: phi*prev < 0 is false when a
    // sample lands exactly on 0, and 4.5 dropped whole swings that way.
    if((phi >= 0) != (decayPhiPrev >= 0) && fabsf(phi) < 1.0f){ decayCrossed=true; decayPeak=0; }
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
      decayTms[decayN] = millis(); decayAmp[decayN] = decayPeak;
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
  } else if(mode==SWINGUP && calFreeTick(phi)){
    // live pole measurement in progress: cart stopped, pole swinging freely
  } else if(mode==SWINGUP){
    // how much the pole cares about cart motion right now
    float sens = fabsf(wF*cosf(thF));
    if(sens>sensPk) sensPk=sens;
    sensPk *= 0.9994f;                      // decays over ~3 s, tracks the growth
    if(sensPk<0.05f) sensPk=0.05f;          // a high floor here stops it ever starting

    // Near the top, hand over from full-power pumping to smooth energy control.
    // Hysteresis: only go back to pumping if the energy has fallen well away.
    const float GL = G*L;                   // full swing energy - thresholds scale with it
    if(!ecOn && E > EN_SWITCH*GL){ ecOn=true; vCmd=vF; ecSinceMs=millis(); }
    else if(ecOn && E < EN_SW_BACK*GL) ecOn=false;

    // Below about 55 deg the cart hardly travels, so there is no rail to save:
    // pump flat out and get the swing going. Only start conserving once it is wide.
    bool conserve = (E > EN_CONSERVE*GL);
    if(ecOn){
      // --- learn the target from what happens at the top, not from the energy
      //     reading: fell back short -> raise it by the shortfall; went over
      //     the top uncaught -> lower it by the excess. (Sign compares count
      //     zero as positive, so a sample landing exactly on 0 is not missed.)
      bool turned  = (wF >= 0) != (wPrevSign >= 0);
      bool overTop = (thF >= 0) != (thLast >= 0);
      if(turned && fabsf(thF) < PI/2)  eBias += LEARN * (GL/2.0f)*(1.0f - cosf(thF));
      if(overTop && fabsf(thF) < 0.5f) eBias -= LEARN * (L*L/6.0f)*wF*wF;
      eBias = constrain(eBias, -EN_BIAS_MAX*GL, EN_BIAS_MAX*GL);
      // --- final approach. dE/dt = -(L/2)*a*w*cos(th), so pushing against
      //     sign(w*cos th) adds energy and pushing with it removes energy. Push
      //     in proportion to the error: adds when short, BRAKES when there is
      //     too much - that is what stops the pole going round and round.
      //     Meanwhile the cart is eased back toward the middle.
      float sg = (wF*cosf(thF) > 0) ? 1.0f : -1.0f;
      float a  = (KE_N/GL)*(E - eBias)*sg - KXE*xF - KVE*vF;
      if(a >  A_LIM) a =  A_LIM;
      if(a < -A_LIM) a = -A_LIM;
      addAccel(a, dt);
      duty = speedToDuty(vCmd);
      pumping = fabsf(E - eBias) > 0.0127f*GL;
    } else if(!conserve || sens > SENS_GATE*sensPk){
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

    if((phi >= 0) != (phiPrev >= 0) && fabsf(phi) < 1.0f && fabsf(wF)>0.4f){ pumps++; lastGain=E-eAtPump; eAtPump=E; }
    phiPrev=phi;

    float fd = fence(duty, xF);
    if(ecOn && fd != duty) vCmd = dutyToSpeed(fd);
    duty = fd;

    // The automatic sign flip only runs while pumping up from low down, and
    // only until a catch has proved the sign. Near the top the energy is HELD,
    // not climbing - 4.4's check could flip a correct sign there.
    if(millis()>signCheckAt){
      if(!signProven && flips<2 && E < EN_SIGNCHK*GL && E < eAtCheck + 0.025f*GL){
        SIGN_THETA=-SIGN_THETA; thPrev=-thPrev; wF=-wF; phiPrev=-phiPrev;
        flips++; Serial.println("# energy not climbing - flipped SIGN_THETA");
      }
      eAtCheck=E; signCheckAt=millis()+3000;
    }
    // holding the energy near the top is progress, not a stall
    if(E > eBest + 0.0127f*GL){ eBest=E; bestAt=millis(); }
    else if(fabsf(E) < EN_NEAR*GL) bestAt=millis();
    if(millis()-bestAt>12000){
      Serial.print("# stalled at amp "); Serial.print(ampDeg,0);
      Serial.print(" deg, E "); Serial.print(eBest,2);
      Serial.print(", cart reached "); Serial.print(xExcursion*1000,0); Serial.println(" mm");
      halt("no progress in 12 s - run 'd', then read the notes at the bottom");
    } else if(ecOn && millis()-ecSinceMs > 45000){
      Serial.print("# near the top for 45 s, never catchable. E "); Serial.print(E,2);
      Serial.print("  learnt target "); Serial.println(eBias,2);
      halt("send the log - the catch window or energy target needs adjusting");
    } else if(catchable()){
      Serial.print("# caught at "); Serial.print(thF/D2R,1);
      Serial.print(" deg, "); Serial.print(wF/D2R,0);
      Serial.print(" dps, E "); Serial.print(E,2);
      Serial.print(", z "); Serial.print(catchZ(),3);
      Serial.print(", cart "); Serial.print(xF*1000,0); Serial.print(" mm");
      Serial.print(drops ? ", after drop " : ", first try"); if(drops) Serial.print(drops);
      Serial.println();
      mode=BALANCE; signProven=true; vCmd=vF; balStartMs=millis(); held=false;
      duty=0;                       // the balance law takes over on the next tick
    }
    if(mode==SWINGUP) drive(duty);
    wPrevSign=wF;

  // ======================= BALANCE =======================================
  } else if(mode==BALANCE){
    // The LQR asks for an acceleration; the motor delivers speed. Integrate
    // the demand into vCmd and drive that. 4.4 used the demand as duty, and
    // the cart crept off to the fence while the pole stayed up.
    float aCmd;
    if(balCtl == CTL_ESN)           aCmd = esnDot(ESN_WC);                     // network drives
    else if(balCtl == CTL_ESN_PRED) aCmd = KX*xF + KV*vF + KTH*esnPredAngle() + KW*esnPredRate();
    else                            aCmd = KX*xF + KV*vF + KTH*thF + KW*wF;   // plain 4.8 LQR
    addAccel(aCmd, dt);
    duty = speedToDuty(vCmd);
    float fd = fence(duty, xF);
    if(fd != duty) vCmd = dutyToSpeed(fd);
    duty = fd;
    if(fabsf(xF) > X_END) halt("cart past the mechanical end - check the homing");
    else if(fabsf(thF)/D2R>GIVEUP_DEG){
      drops++;
      Serial.print("# dropped it after "); Serial.print((millis()-balStartMs)/1000.0f,1);
      Serial.print(" s - drop "); Serial.print(drops); Serial.print(" of "); Serial.println(MAX_DROPS);
      if(drops >= MAX_DROPS) halt("dropped 5 times without a 3 s hold - check the log");
      else startSwing(false);       // recover: brakes or tops up the energy, no kick
    } else {
      if(!held && millis()-balStartMs > HOLD_MS){
        held=true; drops=0;         // a real catch - the count starts again
        Serial.println("# holding it");
      }
      drive(duty);
    }
    wPrevSign=wF;

  } else { drive(0); wPrevSign=wF; }
  thLast = thF;

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
      Serial.print(calPhase==CAL_FREE ? "  MEASURE" : ecOn ? "  TOP" : pumping ? "  PUMP" : "  centre");
      if(ecOn){ Serial.print("  learnt "); Serial.print(eBias/(G*L),3); }
      Serial.print("  x ");    Serial.print(xF*1000,0);
      Serial.print("/");       Serial.print(xExcursion*1000,0); Serial.print("mm");
      if(ecOn){ Serial.print("  z "); Serial.print(catchZ(),2); }
    }
    if(mode==BALANCE){
      Serial.print("  th ");   Serial.print(thF/D2R,1);
      Serial.print("  x ");    Serial.print(xF*1000,0); Serial.print("mm");
      Serial.print("  held "); Serial.print((millis()-balStartMs)/1000.0f,1); Serial.print("s");
      Serial.print("  ["); Serial.print(balCtlName());
      Serial.print("  ESN predicts th "); Serial.print(esnPredAngle()/D2R,1); Serial.print("]");
    }
    if(mode==IDLE || mode==DECAY){
      // live encoders: cart position, and the pole's deflection from hanging
      // (phi is MINUS the deflection - see THE SIGN TRAP)
      Serial.print("  x ");    Serial.print(xF*1000,0); Serial.print("mm");
      Serial.print("  pole "); Serial.print(-phi/D2R,1); Serial.print("deg");
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
 *  TOP        (4.5) final approach: smooth energy control. E should settle near
 *             +0.05 and z should come close to 0 as the pole nears the top.
 *  x a/b      a = where the cart is now, b = the furthest it has been.
 *
 *   BAL    amp 180  half 61ms  E -0.01/-3.94  th 0.4  x -12mm  held 6.3s
 *
 *  th         angle from upright. Should hover within a degree or two.
 *  x          cart position. Should stay near 0 - 4.4's cart crept to the fence.
 *  held       time since the catch. "# holding it" prints at 3 s.
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
