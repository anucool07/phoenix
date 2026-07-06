/*
  ESP32 + L298N Car — Manual (WiFi) + Automatic (Line Follow / Obstacle Avoid)
  ------------------------------------------------------------------------------
  Combines the WiFi phone-controlled car with the IR line-following /
  ultrasonic obstacle-avoidance car. A button on the web page toggles
  between MANUAL and AUTO mode.

  CHANGES IN THIS VERSION (wall-follow lock-in):
    - When the IR sensors report l==0, m==0, r==0 (all sensors off the
      line), the car now checks BOTH the left and right ultrasonic
      distances. If both are under WALL_ENTRY_DIST (20 cm), it enters
      wallFollowLoop() — a locked-in ultrasonic wall-following mode
      that runs ultra() over and over and NEVER re-checks the IR line
      sensors again, no matter what they read.
    - If the left/right distances are NOT both under the threshold in
      that all-black condition, it just does Back(), same as before.
    - CRITICAL FIX vs. a plain while(1): a bare while(1) would freeze
      the ESP32's WiFi server and you could never switch back to
      MANUAL. wallFollowLoop() instead calls server.handleClient()
      on every iteration (so the "switch to manual" WiFi request can
      actually be received) and uses the existing autoMode flag as
      its loop condition. The instant the phone sends /mode?state=0,
      handleMode() flips autoMode to false, and wallFollowLoop() exits
      on its very next check — motors are stopped immediately and
      control returns to manual mode.
    - Everything else (manual D-pad control, speed slider, line-follow
      logic, ultra() itself) is unchanged from the previous version.

  Wiring (L298N -> ESP32):
    ENA / pwmLeft            -> GPIO 14
    IN1 / leftMotorForward   -> GPIO 27
    IN2 / leftMotorBackward  -> GPIO 26
    ENB / pwmRight           -> GPIO 25
    IN3 / rightMotorForward  -> GPIO 33
    IN4 / rightMotorBackward -> GPIO 32

    (ENA/IN1/IN2 and pwmLeft/leftMotorForward/leftMotorBackward are the
     SAME physical pins — the manual code and the auto code each use
     their own function/variable names for them.)

  IR line sensors:
    irLeftPin  -> GPIO 36
    irMidPin   -> GPIO 35
    irRightPin -> GPIO 34

  Ultrasonic sensors:
    TRIG_LEFT   -> GPIO 18   ECHO_LEFT   -> GPIO 2
    TRIG_CENTER -> GPIO 16   ECHO_CENTER -> GPIO 17
    TRIG_RIGHT  -> GPIO 5    ECHO_RIGHT  -> GPIO 15

    L298N OUT1/OUT2 -> Motor A (left)
    L298N OUT3/OUT4 -> Motor B (right)
    L298N GND       -> ESP32 GND (shared with battery GND)
    L298N 12V/VCC   -> Battery +
*/

#include <WiFi.h>
#include <WebServer.h>
#include <NewPing.h>

// =====================================================================
//  MANUAL-MODE PIN DEFINITIONS (from WiFi car sketch)
// =====================================================================
#define ENA 14
#define IN1 27
#define IN2 26
#define ENB 25
#define IN3 33
#define IN4 32

// PWM settings (manual mode)
const int pwmFreq       = 5000;
const int pwmResolution = 8; // 0-255

// =====================================================================
//  SET YOUR MOTOR SPEEDS HERE (manual mode)
// =====================================================================
int LEFT_SPEED  = 200;  // Base speed for left  motor (0-255)
int RIGHT_SPEED = 188;  // Base speed for right motor (0-255)

// Turn sensitivity: fraction of base speed applied to the inside wheel
// 0.0 = pivot turn (maximum),  1.0 = no turn at all
const float TURN_BIAS = 0.5;
// =====================================================================

// ---------- WiFi AP credentials ----------
const char* ssid     = "A1";
const char* password = "12345678"; // min 8 chars

WebServer server(80);

// ---------- Active-command bitmask (manual mode) ----------
// Bits: 0=fwd, 1=back, 2=left, 3=right
volatile uint8_t activeCmd = 0;

// ---------- Mode flag ----------
// false = MANUAL (WiFi control), true = AUTO (line follow / obstacle avoid)
volatile bool autoMode = false;

// =====================================================================
//  AUTO-MODE PIN DEFINITIONS (from line-follow / obstacle-avoid sketch)
// =====================================================================

// IR Sensor Pins
const int irLeftPin  = 36;
const int irMidPin   = 35;
const int irRightPin = 34;

// UltraSonic Pins
#define TRIG_LEFT    18
#define ECHO_LEFT    2

#define TRIG_CENTER  16
#define ECHO_CENTER  17

#define TRIG_RIGHT   5
#define ECHO_RIGHT   15

#define MAX_DISTANCE 200

// Distance (cm) both left & right ultrasonics must be under, while all
// three IR sensors read 0, to lock the car into wall-follow mode.
#define WALL_ENTRY_DIST 30

NewPing sonarLeft  (TRIG_LEFT,   ECHO_LEFT,   MAX_DISTANCE);
NewPing sonarCenter(TRIG_CENTER, ECHO_CENTER, MAX_DISTANCE);
NewPing sonarRight (TRIG_RIGHT,  ECHO_RIGHT,  MAX_DISTANCE);

// Motor Driver Pins (auto mode names — same physical pins as ENA/IN1..IN4 above)
const int leftMotorForward  = 27;
const int leftMotorBackward = 26;
const int rightMotorForward = 33;
const int rightMotorBackward = 32;
const int pwmLeft  = 14;
const int pwmRight = 25;

int lms = 200; //trimmed left motor speed
int rms = 180;
int turnSpeed = 100;

const int lms2 = 170;
const int rms2 = 150;
const int turnSpeed2 = 100;
const int sharpTurnThreshold = 28;

// ---------- Manual-mode motor helpers ----------
void setMotorA(int spd, bool forward) {
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);
  ledcWrite(ENA, abs(spd));
}

void setMotorB(int spd, bool forward) {
  digitalWrite(IN3, forward ? HIGH : LOW);
  digitalWrite(IN4, forward ? LOW  : HIGH);
  ledcWrite(ENB, abs(spd));
}

void stopMotors() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ---------- Apply combined command bitmask (manual mode) ----------
// This runs every time the phone sends an updated "held buttons" set.
// If nothing is held (cmd == 0), motors are stopped immediately.
// Motors only spin for as long as a direction button is actively held.
void applyMotors(uint8_t cmd) {
  bool fwd   = cmd & 0x01;
  bool back  = cmd & 0x02;
  bool left  = cmd & 0x04;
  bool right = cmd & 0x08;

  // Cancel opposites
  if (fwd  && back)  { fwd  = false; back  = false; }
  if (left && right) { left = false; right = false; }

  if (!fwd && !back && !left && !right) {
    stopMotors();
    return;
  }

  // Determine base direction
  bool goFwd = fwd || (!back); // forward unless only back is pressed
  if (back && !fwd) goFwd = false;

  int lSpd = LEFT_SPEED;
  int rSpd = RIGHT_SPEED;

  // Apply turn bias
  if (left) {
    lSpd = (int)(LEFT_SPEED  * TURN_BIAS);
  }
  if (right) {
    rSpd = (int)(RIGHT_SPEED * TURN_BIAS);
  }

  // If only turning (no fwd/back), do a gentle pivot
  if ((left || right) && !fwd && !back) {
    if (left) {
      setMotorA(lSpd, false);  // left wheel backward
      setMotorB(rSpd, true);   // right wheel forward
    } else {
      setMotorA(lSpd, true);   // left wheel forward
      setMotorB(rSpd, false);  // right wheel backward
    }
    return;
  }

  setMotorA(lSpd, goFwd);
  setMotorB(rSpd, goFwd);
}

// =====================================================================
//  AUTO-MODE FUNCTIONS (line follow / obstacle avoid)
// =====================================================================

void runAutoMode() {
  int irLeft  = analogRead(irLeftPin);
  int irMid   = analogRead(irMidPin);
  int irRight = analogRead(irRightPin);

  int threshold = 1000;

  int l = (irLeft  < threshold) ? 1 : 0; //White color = 1
  int m = (irMid   < threshold) ? 1 : 0;
  int r = (irRight < threshold) ? 1 : 0;

  if((l == 0) && (m == 1) && (r == 0)) forward();
  if((l == 0) && (m == 0) && (r == 1)) sharpRight();
  if((l == 0) && (m == 1) && (r == 1)) sharpRight();
  if((l == 1) && (m == 0) && (r == 0)) sharpLeft();
  if((l == 1) && (m == 1) && (r == 0)) sharpLeft();
  if((l == 1) && (m == 1) && (r == 1)) ultra();

  if((l == 0) && (m == 0) && (r == 0)) {
    int LD = sonarLeft.ping_cm();
    int RD = sonarRight.ping_cm();
    int leftDist  = (LD == 0) ? MAX_DISTANCE : LD;
    int rightDist = (RD == 0) ? MAX_DISTANCE : RD;

    if (leftDist < WALL_ENTRY_DIST && rightDist < WALL_ENTRY_DIST) {
      // Lock into pure ultrasonic wall-following. This call does not
      // return until the phone switches the mode to MANUAL.
      wallFollowLoop();
    } else {
      Back();
    }
  }
}

// ---------------------------------------------------------------------
// Ultrasonic wall-following "lock-in" mode.
// Once entered, IR line sensors are ignored completely — the car stays
// in pure ultrasonic wall-following (ultra()) forever, in a tight loop,
// UNTIL the phone switches the mode to MANUAL over WiFi (autoMode -> false).
//
// IMPORTANT: a plain while(1) here would also freeze the WiFi server,
// since nothing would ever call server.handleClient() again — the
// "/mode?state=0" request would never be received or processed, and
// you'd be stuck in wall mode permanently. Instead:
//   - we pump server.handleClient() every iteration so incoming
//     WiFi requests (including the mode switch) are still handled,
//   - we use the existing autoMode flag (updated by handleMode()) as
//     the loop condition, so the loop exits the instant it's flipped.
// ---------------------------------------------------------------------
void wallFollowLoop() {
  Serial.println("Entering WALL-FOLLOW lock-in mode");
  while (autoMode) {
    server.handleClient();   // keep servicing WiFi so mode-switch can arrive
    if (!autoMode) break;    // switched to manual mid-cycle -> bail out now
    ultra();                 // pure ultrasonic wall-following, no IR checks
    delay(20);
  }
  stopMotors();               // safety stop the instant we drop to manual
  Serial.println("Exiting WALL-FOLLOW mode -> MANUAL");
}

void ultra() {
  int CD = sonarCenter.ping_cm();
  int LD = sonarLeft.ping_cm();
  int RD = sonarRight.ping_cm();
  int leftDist;
  int rightDist;
  int centerDist;
  if(CD == 0){
   centerDist = 40;
  } else {
   centerDist = CD;
  }
  if(LD == 0){
    leftDist = 40;
  } else {
    leftDist = LD;
  }
  if(RD == 0){
    rightDist = 40;
  } else {
    rightDist = RD;
  }
  Serial.print("L:");
  Serial.print(leftDist);
  Serial.print(" C:");
  Serial.print(centerDist);
  Serial.print(" R:");
  Serial.println(rightDist);

  if (centerDist < sharpTurnThreshold) {
    if (leftDist > rightDist) {
      sharpLeft2();
      Serial.println("Sharp left");
    } else {
      sharpRight2();
      Serial.println("Sharp Right");
    }
  } else {
    if (leftDist > rightDist) {
      slightLeft2();
      Serial.println("Slighleft");
    } else if (rightDist > leftDist) {
      slightRight2();
      Serial.println("SlightRIght");
    } else {
      moveForward2();
      Serial.println("Moving forward");
    }
  }
}

void forward()
{
  ledcWrite(pwmLeft, lms);
  ledcWrite(pwmRight, rms);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void sharpRight()
{
  ledcWrite(pwmLeft, rms);
  ledcWrite(pwmRight, turnSpeed);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
}

void slightRight()
{
  ledcWrite(pwmLeft, lms);
  ledcWrite(pwmRight, 70);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void sharpLeft()
{
  ledcWrite(pwmLeft, turnSpeed);
  ledcWrite(pwmRight, rms);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void slightLeft()
{
  ledcWrite(pwmLeft, 85);
  ledcWrite(pwmRight, rms);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void sharpLeft2() {
  ledcWrite(pwmLeft, turnSpeed2-20);
  ledcWrite(pwmRight, rms2);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void slightLeft2() {
  ledcWrite(pwmLeft, 95);
  ledcWrite(pwmRight, rms2);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void sharpRight2() {
  ledcWrite(pwmLeft, rms2-20);
  ledcWrite(pwmRight, turnSpeed2-20);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

void slightRight2() {
  ledcWrite(pwmLeft, lms2);
  ledcWrite(pwmRight, 70);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void moveForward2() {
  ledcWrite(pwmLeft, lms2);
  ledcWrite(pwmRight, rms2);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void Back()
{
  ledcWrite(pwmLeft, lms-22);
  ledcWrite(pwmRight, rms-10);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

// ---------- Web page ----------
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Car</title>
  <style>
    * { box-sizing: border-box; user-select: none; -webkit-user-select: none; }
    body {
      font-family: sans-serif; text-align: center;
      background: #111; color: #eee;
      margin: 0; padding: 20px;
      touch-action: none;
      overflow: hidden;
    }
    h2 { margin-bottom: 10px; }

    #modeBtn {
      width: 220px; height: 50px;
      font-size: 18px;
      border-radius: 10px;
      border: none;
      background: #27ae60;
      color: white;
      cursor: pointer;
      margin-bottom: 15px;
    }
    #modeBtn.auto { background: #8e44ad; }

    /* Plus-sign (D-pad) layout */
    .dpad {
      display: grid;
      grid-template-columns: 90px 90px 90px;
      grid-template-rows: 90px 90px 90px;
      gap: 10px;
      justify-content: center;
      margin: 30px auto;
      width: max-content;
    }

    button {
      width: 90px; height: 90px;
      font-size: 28px;
      border-radius: 14px;
      border: none;
      background: #2b6cb0;
      color: white;
      cursor: pointer;
      touch-action: none;
    }
    button.active { background: #1a4e8a; }

    #btn-fwd   { grid-column: 2; grid-row: 1; }
    #btn-left  { grid-column: 1; grid-row: 2; }
    #btn-stop  { grid-column: 2; grid-row: 2; background: #c0392b; font-size: 16px; }
    #btn-stop.active { background: #8e2317; }
    #btn-right { grid-column: 3; grid-row: 2; }
    #btn-back  { grid-column: 2; grid-row: 3; }

    .speed-section { margin-top: 20px; }
    .slider { width: 70%; }
  </style>
</head>
<body>
  <h2>ESP32 Car</h2>

  <button id="modeBtn" onclick="toggleMode()">Mode: MANUAL</button>

  <div class="dpad">
    <button id="btn-fwd"   data-cmd="fwd">&#8593;</button>
    <button id="btn-left"  data-cmd="left">&#8592;</button>
    <button id="btn-stop">STOP</button>
    <button id="btn-right" data-cmd="right">&#8594;</button>
    <button id="btn-back"  data-cmd="back">&#8595;</button>
  </div>

  <div class="speed-section">
    <p>Speed: <span id="spdVal">200</span></p>
    <input type="range" min="80" max="255" value="200" class="slider"
           id="spd" oninput="setSpeed(this.value)">
  </div>

  <script>
    // Track which commands are currently active
    const held = { fwd: false, back: false, left: false, right: false };

    function sendCmd() {
      const active = Object.entries(held)
        .filter(([k, v]) => v)
        .map(([k]) => k)
        .join(',');
      fetch('/cmd?keys=' + encodeURIComponent(active));
    }

    function clearAllHeld() {
      Object.keys(held).forEach(k => held[k] = false);
      document.querySelectorAll('[data-cmd]').forEach(btn => btn.classList.remove('active'));
      sendCmd();
    }

    document.querySelectorAll('[data-cmd]').forEach(btn => {
      const cmd = btn.dataset.cmd;

      btn.addEventListener('pointerdown', e => {
        e.preventDefault();
        btn.setPointerCapture(e.pointerId);
        held[cmd] = true;
        btn.classList.add('active');
        sendCmd();
      });

      const release = e => {
        held[cmd] = false;
        btn.classList.remove('active');
        sendCmd();
      };
      btn.addEventListener('pointerup',     release);
      btn.addEventListener('pointercancel', release);
      btn.addEventListener('pointerleave',  release);
    });

    // Stop button: immediately clears every held direction and stops motors
    const stopBtn = document.getElementById('btn-stop');
    stopBtn.addEventListener('pointerdown', e => {
      e.preventDefault();
      stopBtn.classList.add('active');
      clearAllHeld();
    });
    const stopRelease = () => stopBtn.classList.remove('active');
    stopBtn.addEventListener('pointerup',     stopRelease);
    stopBtn.addEventListener('pointercancel', stopRelease);
    stopBtn.addEventListener('pointerleave',  stopRelease);

    // Speed slider
    function setSpeed(val) {
      document.getElementById('spdVal').innerText = val;
      fetch('/speed?val=' + val);
    }

    // Mode toggle (Manual <-> Auto)
    let auto = false;
    function toggleMode() {
      auto = !auto;
      fetch('/mode?state=' + (auto ? 1 : 0));
      const btn = document.getElementById('modeBtn');
      btn.innerText = 'Mode: ' + (auto ? 'AUTO' : 'MANUAL');
      btn.classList.toggle('auto', auto);
    }

    // Safety: release everything if page loses focus (tab switch, lock screen)
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) {
        clearAllHeld();
      }
    });
    window.addEventListener('blur', () => {
      clearAllHeld();
    });
  </script>
</body>
</html>
)rawliteral";

// ---------- Handlers ----------
void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

// Single endpoint that receives the full active key set (manual mode)
void handleCmd() {
  uint8_t mask = 0;
  if (server.hasArg("keys")) {
    String keys = server.arg("keys");
    if (keys.indexOf("fwd")   >= 0) mask |= 0x01;
    if (keys.indexOf("back")  >= 0) mask |= 0x02;
    if (keys.indexOf("left")  >= 0) mask |= 0x04;
    if (keys.indexOf("right") >= 0) mask |= 0x08;
  }
  activeCmd = mask;
  if (!autoMode) {
    applyMotors(activeCmd);
  }
  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  if (server.hasArg("val")) {
    int v = server.arg("val").toInt();
    LEFT_SPEED  = v;
    RIGHT_SPEED = v;
  }
  server.send(200, "text/plain", "OK");
}

// Toggle between MANUAL and AUTO mode
void handleMode() {
  if (server.hasArg("state")) {
    autoMode = server.arg("state").toInt() == 1;
    stopMotors();      // safety stop on mode switch
    activeCmd = 0;
  }
  server.send(200, "text/plain", autoMode ? "AUTO" : "MANUAL");
}

void setup() {
  Serial.begin(115200);

  // Motor pins (shared physical pins between manual/auto naming)
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcAttach(ENA, pwmFreq, pwmResolution);
  ledcAttach(ENB, pwmFreq, pwmResolution);

  stopMotors();

  // IR sensor pins (auto mode)
  pinMode(irLeftPin, INPUT);
  pinMode(irMidPin, INPUT);
  pinMode(irRightPin, INPUT);

  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP()); // usually 192.168.4.1

  server.on("/",      handleRoot);
  server.on("/cmd",   handleCmd);
  server.on("/speed", handleSpeed);
  server.on("/mode",  handleMode);
  server.begin();
}

void loop() {
  server.handleClient();

  if (autoMode) {
    runAutoMode();
  }
}
