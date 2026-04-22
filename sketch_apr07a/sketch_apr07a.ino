#include <Stepper.h>
#include <deque>
#include <cmath>
#include <algorithm>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

struct FuzzyResult { float stopTime; int measurements; };

const int CAM_WIDTH  = 1600;
const int CAM_HEIGHT = 1200;
const int CAL_X_OFFSET = 0;
const int CAL_Y_OFFSET = 0;

const int STEPS_PER_REV = 2048;
const float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0;

const int motorXPins[4] = {13, 12, 14, 27};
const int motorYPins[4] = {26, 33, 25, 32};

Stepper motorX(STEPS_PER_REV, motorXPins[0], motorXPins[1], motorXPins[2], motorXPins[3]);
Stepper motorY(STEPS_PER_REV, motorYPins[0], motorYPins[1], motorYPins[2], motorYPins[3]);

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

const int JOY_X = 34;
const int JOY_Y = 35;
const int JOY_SW = 23;

long currentStepsX = 90 * STEPS_PER_DEGREE;
long currentStepsY = 90 * STEPS_PER_DEGREE;
int lastSW = HIGH;
unsigned long lastMoveTime = 0;
const int MOVE_DELAY = 15;
unsigned long lastTempDisplay = 0;
const int TEMP_DISPLAY_INTERVAL = 2000;

const int MAX_POINTS = 20;
const int WINDOW_SIZE = 5;
std::deque<float> tempHistory[MAX_POINTS];

int getAngleX() { return currentStepsX / STEPS_PER_DEGREE; }
int getAngleY() { return currentStepsY / STEPS_PER_DEGREE; }

void setStepsFromAngle(int angleX, int angleY) {
    currentStepsX = angleX * STEPS_PER_DEGREE;
    currentStepsY = angleY * STEPS_PER_DEGREE;
}

float getTemperature() {
    float objTemp = mlx.readObjectTempC();
    if (isnan(objTemp)) {
        Wire.begin(21, 22);
        if (!mlx.begin()) return -273.15;
        delay(50);
        objTemp = mlx.readObjectTempC();
    }
    return objTemp;
}

void printTemperatureInfo() {
    float objTemp = mlx.readObjectTempC();
    float ambTemp = mlx.readAmbientTempC();
    if (!isnan(objTemp) && !isnan(ambTemp)) {
        Serial.print("TEMP_INFO: Object=");
        Serial.print(objTemp, 2);
        Serial.print("°C, Ambient=");
        Serial.print(ambTemp, 2);
        Serial.println("°C");
    }
}

float errorSmall(float d) { return (d<=1.0)?1.0:(d<=2.0)?(2.0-d):0.0; }
float errorMedium(float d){ return (d<=1.0)?d:(d<=3.0)?1.0:(d<=5.0)?(5.0-d)/2.0:0.0; }
float errorLarge(float d) { return (d<=3.0)?0.0:(d<=5.0)?(d-3.0)/2.0:1.0; }
float trendFalling(float s){ return (s<=-0.3)?1.0:(s<=0.0)?(-s)/0.3:0.0; }
float trendStable(float s) { return (s<=-0.2)?0.0:(s<=0.0)?(s+0.2)/0.2:(s<=0.2)?(0.2-s)/0.2:0.0; }
float trendRising(float s) { return (s<=0.0)?0.0:(s<=0.3)?s/0.3:1.0; }

float calculateTrend(const std::deque<float>& w) {
    int n = w.size();
    if(n<2) return 0.0;
    float sX=0, sY=0, sXY=0, sX2=0;
    for(int i=0; i<n; i++){
        sX += i;
        sY += w[i];
        sXY += i * w[i];
        sX2 += i * i;
    }
    float d = n*sX2 - sX*sX;
    float s = (abs(d)<0.001) ? 0.0 : (n*sXY - sX*sY)/d;

    if (abs(s) < 0.05){
      s = 0.0;
      }
    return s;
}

FuzzyResult fuzzyAnalyze(float cur, float tgt, std::deque<float>& win) {
    FuzzyResult r;
    float d = abs(cur - tgt);
    win.push_back(cur);
    if(win.size() > WINDOW_SIZE) win.pop_front();

    float s = calculateTrend(win);
    float mS = errorSmall(d), mM = errorMedium(d), mL = errorLarge(d);
    float mF = trendFalling(s), mSt = trendStable(s), mR = trendRising(s);

    float wS = fmin(mS, mSt) + fmin(mS, mF);
    float wM = fmin(mM, mSt) + fmin(mM, mF) + fmin(mL, mF) + fmin(mS, mR);
    float wL = fmin(mM, mR) + fmin(mL, mSt) + fmin(mL, mR);
    float tot = wS + wM + wL;
    
    if(tot < 0.01){
        r.stopTime = 2.0;
        r.measurements = 2;
        return r;
    }

    r.stopTime = (wS*0.5 + wM*2.0 + wL*5.0) / tot;
    r.measurements = constrain((int)(r.stopTime * 2.0), 1, 30);
    return r;
}

int angToPixX(int a) { return constrain(map(a, 0, 180, 0, CAM_WIDTH), 0, CAM_WIDTH); }
int angToPixY(int a) { return constrain(map(a, 0, 180, 0, CAM_HEIGHT), 0, CAM_HEIGHT); }
int pixToAngX(int p) { return constrain(map(p, 0, CAM_WIDTH, 0, 180), 0, 180); }
int pixToAngY(int p) { return constrain(map(p, 0, CAM_HEIGHT, 0, 180), 0, 180); }

void moveToSync(int targetX, int targetY) {
    targetX = constrain(targetX, 0, 180);
    targetY = constrain(targetY, 0, 180);
    long targetStepsX = targetX * STEPS_PER_DEGREE;
    long targetStepsY = targetY * STEPS_PER_DEGREE;
    long dX = targetStepsX - currentStepsX;
    long dY = targetStepsY - currentStepsY;  // Убрана инверсия для автоматического режима
    long stepsX = abs(dX), stepsY = abs(dY);
    int dirX = (dX >= 0) ? 1 : -1;
    int dirY = (dY >= 0) ? 1 : -1;

    motorX.setSpeed(10);
    motorY.setSpeed(10);

    while (stepsX > 0 || stepsY > 0) {
        if (stepsX > 0) { motorX.step(dirX); stepsX--; }
        if (stepsY > 0) { motorY.step(dirY); stepsY--; }
    }

    motorX.setSpeed(5);
    motorY.setSpeed(5);

    currentStepsX = targetStepsX;
    currentStepsY = targetStepsY;

    Serial.print("POS:");
    Serial.print(getAngleX());
    Serial.print(",");
    Serial.println(getAngleY());
    Serial.println("MOVE_COMPLETE");
}

void processCommand(String cmd) {
    cmd.trim();
    cmd.replace(" ", "");

    if (cmd == "GET_POS") {
        Serial.print("POS:");
        Serial.print(getAngleX());
        Serial.print(",");
        Serial.println(getAngleY());
    }
    else if (cmd == "GET_TEMP") {
        float temp = getTemperature();
        Serial.print("TEMP:");
        Serial.println(temp, 2);
    }
    else if (cmd == "GET_TEMP_INFO") {
        printTemperatureInfo();
    }
    else if (cmd == "CLEAR_HISTORY") {
        for (int i = 0; i < MAX_POINTS; i++) {
            tempHistory[i].clear();
        }
        Serial.println("HISTORY_CLEARED");
    }
    else if (cmd.startsWith("SET_POS:")) {
        int c = cmd.indexOf(',');
        if(c < 0) return;
        int ax = constrain(cmd.substring(8, c).toInt(), 0, 180);
        int ay = constrain(cmd.substring(c + 1).toInt(), 0, 180);
        setStepsFromAngle(ax, ay);
        Serial.print("POS:");
        Serial.print(getAngleX());
        Serial.print(",");
        Serial.println(getAngleY());
    }
    else if (cmd.startsWith("MOVE_TO:")) {
        int c = cmd.indexOf(',');
        if(c < 0) return;
        int tx = cmd.substring(8, c).toInt();
        int ty = cmd.substring(c + 1).toInt();
        moveToSync(tx, ty);
    }
    else if (cmd.startsWith("MEASURE:")) {
        int c = cmd.indexOf(',');
        if(c < 0) return;
        int id = cmd.substring(8, c).toInt();
        float tgt = cmd.substring(c + 1).toFloat();

        if (id < 0 || id >= MAX_POINTS) {
            Serial.println("MEASURE_ERROR:INVALID_ID");
            Serial.println("MEASURE_COMPLETE");
            return;
        }

        std::deque<float>& win = tempHistory[id];

        float cur = getTemperature();
        if (cur < -100) {
            Serial.print("MEASURE_ERROR:");
            Serial.println(id);
            Serial.println("MEASURE_COMPLETE");
            return;
        }

        FuzzyResult r = fuzzyAnalyze(cur, tgt, win);

        Serial.print("MEASURE_RESULT:");
        Serial.print(id);
        Serial.print(",");
        Serial.print(cur, 2);
        Serial.print(",");
        Serial.print(tgt, 1);
        Serial.print(",");
        Serial.print(r.stopTime, 1);
        Serial.print(",");
        Serial.println(r.measurements);

        for(int i = 0; i < r.measurements; i++){
            delay(500);
            float temp = getTemperature();
            if (temp > -100) {
                Serial.print("MEASURE_DATA:");
                Serial.print(id);
                Serial.print(",");
                Serial.print(i);
                Serial.print(",");
                Serial.println(temp, 2);
            }
        }

        Serial.println("MEASURE_COMPLETE");
        Serial.print("MEASURE_FINAL_TEMP:");
        Serial.print(id);
        Serial.print(",");
        Serial.println(getTemperature(), 2);
    }
}

void setup() {
    Serial.begin(115200);
    while(!Serial);

    Wire.begin(21, 22);
    if (!mlx.begin()) {
        Serial.println("ERROR: MLX90614 not found!");
    } else {
        Serial.println("MLX90614 OK");
    }

    motorX.setSpeed(5);
    motorY.setSpeed(5);

    pinMode(JOY_SW, INPUT_PULLUP);
    pinMode(JOY_X, INPUT);
    pinMode(JOY_Y, INPUT);

    Serial.print("POS:");
    Serial.print(getAngleX());
    Serial.print(",");
    Serial.println(getAngleY());
    Serial.println("ESP32_READY");
}

void loop() {
    if(Serial.available()){
        processCommand(Serial.readStringUntil('\n'));
        return;
    }

    int jX = analogRead(JOY_X);
    int jY = analogRead(JOY_Y);
    int jSW = digitalRead(JOY_SW);
    const int DZ = 500;
    unsigned long now = millis();

    if(now - lastTempDisplay > TEMP_DISPLAY_INTERVAL) {
        float objTemp = mlx.readObjectTempC();
        float ambTemp = mlx.readAmbientTempC();
        if (!isnan(objTemp) && !isnan(ambTemp)) {
            Serial.print("LIVE_TEMP: Object=");
            Serial.print(objTemp, 2);
            Serial.print("°C, Ambient=");
            Serial.print(ambTemp, 2);
            Serial.print("°C, Position=(");
            Serial.print(getAngleX());
            Serial.print(",");
            Serial.print(getAngleY());
            Serial.println(")");
        }
        lastTempDisplay = now;
    }

    if(now - lastMoveTime > MOVE_DELAY){
        int dX = 0, dY = 0;
        if(jX < 2048 - DZ) dX = -1;
        else if(jX > 2048 + DZ) dX = 1;
        if(jY < 2048 - DZ) dY = -1;       // Инверсия для ручного управления
        else if(jY > 2048 + DZ) dY = 1;

        if(dX != 0) { motorX.step(dX); currentStepsX += dX; }
        if(dY != 0) { motorY.step(dY); currentStepsY += dY; }

        if(dX != 0 || dY != 0){
            Serial.print("POS:");
            Serial.print(getAngleX());
            Serial.print(",");
            Serial.println(getAngleY());
            lastMoveTime = now;
        }
    }

    if(jSW == LOW && lastSW == HIGH){
        Serial.println("BUTTON_PRESS");
        Serial.print("BUTTON_TEMP: ");
        Serial.println(getTemperature(), 2);
        printTemperatureInfo();
        delay(200);
    }
    lastSW = jSW;
    delay(10);
}
