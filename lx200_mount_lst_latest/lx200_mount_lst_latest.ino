#include <AccelStepper.h>
#include <math.h>

// ============================================================================
//  Arduino LX200 Mount — LST (real-time) model
//  RA motor position encodes HOUR ANGLE. Reported RA = LST - HA.
//  Time & longitude are captured from Ekos on connect (no RTC chip needed).
// ============================================================================

// Shared Enable Pin on CNC Shield v3
const int EN_PIN = 8;

// Pin Definitions
const int Y_STEP_PIN = 3; // Dec Axis
const int Y_DIR_PIN  = 6;
const int Z_STEP_PIN = 4; // RA Axis
const int Z_DIR_PIN  = 7;

AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN); // Dec
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN); // RA

// --- CALIBRATION (must be TRUE values now — SIDEREAL_SPEED depends on them) ---
const float STEPS_PER_DEG_DEC  = 3733.0 * 2 / 1.5;
const float STEPS_PER_DEG_RA  = 3733.0 * 2;
const float STEPS_PER_HOUR_RA  = STEPS_PER_DEG_RA * 15.0;   // steps per RA-hour

// Direction knobs: flip if an axis moves the wrong way.
//   RA_DIR : if the mount TRACKS backwards (star drifts faster), flip to +1
//   DEC_DIR: if North/South are inverted, flip to -1
const int   RA_DIR  = 1;
const int   DEC_DIR = 1;

// Sidereal advance ratio (LST advances this many RA-hours per solar hour)
const float SIDEREAL_RATIO = 1.0027379f;

// Tracking speed is DERIVED so reported RA stays constant while tracking.
//   steps/sec = STEPS_PER_HOUR_RA * SIDEREAL_RATIO / 3600
const float SIDEREAL_SPEED =
    RA_DIR * (STEPS_PER_HOUR_RA * SIDEREAL_RATIO / 3600.0);

// --- Manual slew rates (Steps / Second) ---
const float SPEED_GUIDE  = 10.0;
const float SPEED_CENTER = 50.0;
const float SPEED_FIND   = 200.0;
const float SPEED_MAX    = 3000.0;

const float SLEW_MAX_SPEED = 4000.0;  // max speed during GOTO run()

// --- Serial / state ---
String inputBuffer = "";
bool   capturing   = false;
float  currentSlewSpeed = SPEED_FIND;

enum MountState { TRACKING, SLEWING };
MountState mountState = TRACKING;

// --- Targets ---
float targetRaHours = 0.0;   // stored as hours; converted to steps at :MS# time
long  targetDecSteps = 0;

// --- Time / site captured from Ekos ---
int   siteYear = 2000, siteMonth = 1, siteDay = 1;
long  siteLocalSec = 0;          // seconds since local midnight, at capture
unsigned long timeBaseMillis = 0;
float utcOffsetToUTC = 0.0;      // :SG  — hours ADDED to local to get UTC
float lonWestDeg = 0.0;          // :Sg  — Meade longitude (WEST-positive, 0..360)
float latDeg = 0.0;              // :St
bool  timeKnown = false;
bool  dateKnown = false;

// ============================================================================
void setup() {
  Serial.begin(9600);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);            // enable drivers (active LOW)

  stepperY.setMaxSpeed(SLEW_MAX_SPEED);
  stepperZ.setMaxSpeed(SLEW_MAX_SPEED);
  stepperY.setAcceleration(400.0);      // REQUIRED for run()/moveTo() gotos
  stepperZ.setAcceleration(400.0);

  stepperZ.setSpeed(SIDEREAL_SPEED);    // RA tracking
  stepperY.setSpeed(0);                 // Dec still
}

void loop() {
  readLX200Serial();

  if (mountState == SLEWING) {
    stepperY.run();
    stepperZ.run();
    if (stepperY.distanceToGo() == 0 && stepperZ.distanceToGo() == 0) {
      mountState = TRACKING;
      stepperZ.setSpeed(SIDEREAL_SPEED);
      stepperY.setSpeed(0);
    }
  } else {
    stepperY.runSpeed();
    stepperZ.runSpeed();
    // No reference hack needed: LST advances at the same rate the RA motor
    // tracks, so RA = LST - HA is naturally constant on a tracked star.
  }
}

// ============================================================================
//  Serial reader (colon-safe + ACK)
// ============================================================================
void readLX200Serial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 0x06) { Serial.print('P'); continue; }   // ACK -> 'P' (polar)

    if (!capturing) {
      if (c == ':') { capturing = true; inputBuffer = ""; }
    } else if (c == '#') {
      inputBuffer.trim();
      parseLX200Command(inputBuffer);
      inputBuffer = "";
      capturing = false;
    } else {
      inputBuffer += c;
    }
  }
}

// ============================================================================
//  Command parser
// ============================================================================
void parseLX200Command(String cmd) {

  // --- Motion start ---
  if (cmd == "Mn" || cmd == "Ms" || cmd == "Me" || cmd == "Mw")
    mountState = TRACKING;

  if      (cmd == "Mn") stepperY.setSpeed(currentSlewSpeed);
  else if (cmd == "Ms") stepperY.setSpeed(-currentSlewSpeed);
  else if (cmd == "Me") stepperZ.setSpeed(SIDEREAL_SPEED + currentSlewSpeed);
  else if (cmd == "Mw") stepperZ.setSpeed(SIDEREAL_SPEED - currentSlewSpeed);

  // --- Motion stop ---
  else if (cmd == "Qn" || cmd == "Qs") stepperY.setSpeed(0);
  else if (cmd == "Qe" || cmd == "Qw") stepperZ.setSpeed(SIDEREAL_SPEED);
  else if (cmd == "Q") {
    mountState = TRACKING;
    stepperY.setSpeed(0);
    stepperZ.setSpeed(SIDEREAL_SPEED);
  }

  // --- Slew rates ---
  else if (cmd == "RG") currentSlewSpeed = SPEED_GUIDE;
  else if (cmd == "RC") currentSlewSpeed = SPEED_CENTER;
  else if (cmd == "RM") currentSlewSpeed = SPEED_FIND;
  else if (cmd == "RS") currentSlewSpeed = SPEED_MAX;

  // --- Current coordinates ---
  else if (cmd == "GR") Serial.print(getFormattedRA()  + "#");
  else if (cmd == "GD") Serial.print(getFormattedDec() + "#");

  // --- Status / version ---
  else if (cmd == "GW")  Serial.print("PT#");
  else if (cmd == "GVP") Serial.print("Arduino Mount#");
  else if (cmd == "GVN") Serial.print("v2.0#");
  else if (cmd == "GVD") Serial.print("Jan 01 2026#");

  // --- Site / time "get" (answered from captured values) ---
  else if (cmd == "Gc") Serial.print("24#");
  else if (cmd == "GM") Serial.print("Home#");
  else if (cmd == "GT") Serial.print("60.1#");
  else if (cmd == "GL") Serial.print(getFormattedLocalTime() + "#");
  else if (cmd == "GS") Serial.print(hoursToHMS(currentLST()) + "#");
  else if (cmd == "GC") Serial.print(getFormattedDate() + "#");
  else if (cmd == "GG") { char b[8]; sprintf(b, "%+03d#", (int)utcOffsetToUTC); Serial.print(b); }
  else if (cmd == "Gt") Serial.print(getFormattedLat() + "#");
  else if (cmd == "Gg") Serial.print(getFormattedLon() + "#");

  // --- Site / time "set" (captured; drive LST) ---
  else if (cmd.startsWith("SL")) {          // local time  HH:MM:SS
    String t = cmd.substring(2); t.trim();
    int h = t.substring(0, 2).toInt();
    int m = t.substring(3, 5).toInt();
    int s = t.substring(6, 8).toInt();
    siteLocalSec  = (long)h * 3600 + (long)m * 60 + s;
    timeBaseMillis = millis();
    timeKnown = true;
    Serial.print("1");
  }
  else if (cmd.startsWith("SC")) {          // date  MM/DD/YY
    String dt = cmd.substring(2); dt.trim();
    siteMonth = dt.substring(0, 2).toInt();
    siteDay   = dt.substring(3, 5).toInt();
    siteYear  = 2000 + dt.substring(6, 8).toInt();
    dateKnown = true;
    // Meade: reply '1' then TWO '#'-terminated strings the driver reads/discards
    Serial.print("1Updating Planetary Data#                        #");
  }
  else if (cmd.startsWith("SG")) {          // UTC offset  sHH.H
    String o = cmd.substring(2); o.trim();
    utcOffsetToUTC = o.toFloat();
    Serial.print("1");
  }
  else if (cmd.startsWith("Sg")) {          // longitude  DDD*MM (west-positive)
    String g = cmd.substring(2); g.trim();
    int deg = g.substring(0, 3).toInt();
    int mn  = g.substring(4, 6).toInt();
    lonWestDeg = deg + mn / 60.0;
    Serial.print("1");
  }
  else if (cmd.startsWith("St")) {          // latitude  sDD*MM
    String t = cmd.substring(2); t.trim();
    int sign = (t.charAt(0) == '-') ? -1 : 1;
    int deg = t.substring(1, 3).toInt();
    int mn  = t.substring(4, 6).toInt();
    latDeg = sign * (deg + mn / 60.0);
    Serial.print("1");
  }
  else if (cmd.startsWith("SM") || cmd.startsWith("SN") ||
           cmd.startsWith("SO") || cmd.startsWith("SP")) {
    Serial.print("1");                      // site names
  }

  // --- Set target RA (store hours; HA computed at :MS#) ---
  else if (cmd.startsWith("Sr")) {
    String raStr = cmd.substring(2); raStr.trim();
    targetRaHours = parseRaToHours(raStr);
    Serial.print("1");
  }
  // --- Set target Dec ---
  else if (cmd.startsWith("Sd")) {
    String decStr = cmd.substring(2); decStr.trim();
    targetDecSteps = parseDecToSteps(decStr);
    Serial.print("1");
  }

  // --- Start slew ---
  else if (cmd == "MS") {
    Serial.print("0");
    float ha = currentLST() - targetRaHours;    // hour angle of target now
    while (ha < -12.0) ha += 24.0;
    while (ha >= 12.0) ha -= 24.0;
    long targetRaSteps = (long)(RA_DIR * ha * STEPS_PER_HOUR_RA);
    stepperZ.moveTo(targetRaSteps);
    stepperY.moveTo(targetDecSteps);
    mountState = SLEWING;
  }

  // --- Distance bars (slew-complete detection) ---
  else if (cmd == "D") {
  if (mountState == SLEWING) Serial.print("\x7f#");  // bar = still moving
  else                       Serial.print("#");       // done
  }
  // Sync: define current position as the target coords (no motion)
  else if (cmd == "CM") {
  // RA -> hour angle at the current sidereal time
  float ha = currentLST() - targetRaHours;
  while (ha < -12.0) ha += 24.0;
  while (ha >= 12.0) ha -= 24.0;
  stepperZ.setCurrentPosition((long)(RA_DIR * ha * STEPS_PER_HOUR_RA));
  stepperY.setCurrentPosition(targetDecSteps);

  stepperZ.setSpeed(SIDEREAL_SPEED);   // setCurrentPosition() zeroed speed; restore tracking

  Serial.print("Synced#");   // any '#'-terminated string; driver just reads it
  }
}

// ============================================================================
//  Local Sidereal Time  (computed from captured UTC + longitude)
// ============================================================================
double julianDate(int Y, int M, int D, double UTh) {
  long a = (14 - M) / 12;
  long y = Y + 4800 - a;
  long m = M + 12 * a - 3;
  long jdn = D + (153 * m + 2) / 5 + 365 * y + y / 4 - y / 100 + y / 400 - 32045;
  return (double)jdn - 0.5 + UTh / 24.0;
}

double normalize24(double h) { h = fmod(h, 24.0); if (h < 0) h += 24.0; return h; }

float currentLST() {
  if (!timeKnown || !dateKnown) return 0.0;

  float elapsedSec = (millis() - timeBaseMillis) / 1000.0;
  double localHours = (siteLocalSec + elapsedSec) / 3600.0;
  double utHours = localHours + utcOffsetToUTC;   // add offset -> UTC

  int y = siteYear, mo = siteMonth, d = siteDay;
  while (utHours >= 24.0) { utHours -= 24.0; d += 1; }  // roll midnight
  while (utHours <  0.0)  { utHours += 24.0; d -= 1; }

  double JD  = julianDate(y, mo, d, utHours);
  double Dj  = JD - 2451545.0;
  double GMST = normalize24(18.697374558 + 24.06570982441908 * Dj);
  double lonEast = -lonWestDeg;                   // Meade is west-positive
  return (float)normalize24(GMST + lonEast / 15.0);
}


// ============================================================================
//  Parse helpers
// ============================================================================
float parseRaToHours(String str) {
  int h = str.substring(0, 2).toInt();
  int m = str.substring(3, 5).toInt();
  int s = 0;
  if (str.length() >= 8) s = str.substring(6, 8).toInt();
  return h + m / 60.0 + s / 3600.0;
}

long parseDecToSteps(String str) {
  int sign = (str.charAt(0) == '-') ? -1 : 1;
  int d = str.substring(1, 3).toInt();
  int m = str.substring(4, 6).toInt();
  int s = 0;
  if (str.length() >= 8) s = str.substring(7, 9).toInt();
  float deg = (d + m / 60.0 + s / 3600.0) * sign;
  return (long)(DEC_DIR * deg * STEPS_PER_DEG_DEC);
}

// ============================================================================
//  Format helpers
// ============================================================================
String hoursToHMS(float hours) {
  while (hours < 0)     hours += 24.0;
  while (hours >= 24.0) hours -= 24.0;
  int h = (int)hours;
  int m = (int)((hours - h) * 60);
  int s = (int)((hours - h - m / 60.0) * 3600);
  char buf[10];
  sprintf(buf, "%02d:%02d:%02d", h, m, s);
  return String(buf);
}

String getFormattedRA() {
  float ha = RA_DIR * stepperZ.currentPosition() / STEPS_PER_HOUR_RA;
  float raH = (timeKnown && dateKnown) ? (currentLST() - ha) : ha;  // fallback pre-connect
  return hoursToHMS(raH);
}

String getFormattedDec() {
  float currentDeg = DEC_DIR * stepperY.currentPosition() / STEPS_PER_DEG_DEC;
  char sign = (currentDeg >= 0) ? '+' : '-';
  currentDeg = fabs(currentDeg);
  int d = (int)currentDeg;
  int m = (int)((currentDeg - d) * 60);
  int s = (int)((currentDeg - d - m / 60.0) * 3600);
  char buf[11];
  sprintf(buf, "%c%02d*%02d'%02d", sign, d, m, s);
  return String(buf);
}

String getFormattedLocalTime() {
  float elapsed = (millis() - timeBaseMillis) / 1000.0;
  return hoursToHMS((siteLocalSec + elapsed) / 3600.0);
}

String getFormattedDate() {
  char b[10];
  sprintf(b, "%02d/%02d/%02d", siteMonth, siteDay, siteYear % 100);
  return String(b);
}

String getFormattedLat() {
  char sign = (latDeg < 0) ? '-' : '+';
  float a = fabs(latDeg);
  int dd = (int)a, mm = (int)((a - dd) * 60);
  char b[10];
  sprintf(b, "%c%02d*%02d", sign, dd, mm);
  return String(b);
}

String getFormattedLon() {
  int dd = (int)lonWestDeg, mm = (int)((lonWestDeg - dd) * 60);
  char b[10];
  sprintf(b, "%03d*%02d", dd, mm);
  return String(b);
}
