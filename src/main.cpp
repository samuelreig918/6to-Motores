#include <Arduino.h>
#include <ESP32Servo.h>

#define POT_T0 34
#define POT_P0 35
#define POT_V0 32
#define POT_M  33
#define SWITCH 4
#define ESC_PIN 27
#define Baudreate 115200
#define Ratio_Pressure 13.5
#define T4 1265.0
#define Number_of_values 122

Servo esc;

// Tabla A-22: T [K], h [kJ/kg], Pr [-]
const float Temperature[Number_of_values] = {
200,210,220,230,240,250,260,270,280,285,290,295,300,305,310,315,320,325,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640,650,660,670,680,690,700,710,720,730,740,750,760,770,780,790,800,820,840,860,880,900,920,940,960,980,1000,1020,1040,1060,1080,1100,1120,1140,1160,1180,1200,1220,1240,1260,1280,1300,1320,1340,1360,1380,1400,1420,1440,1460,1480,1500,1520,1540,1560,1580,1600,1620,1640,1660,1680,1700,1750,1800,1850,1900,1950,2000,2050,2100,2150,2200,2250};

const float Enthalpy[Number_of_values] = {
199.97,209.97,219.97,230.02,240.02,250.05,260.09,270.11,280.13,285.14,290.16,295.17,300.19,305.22,310.24,315.27,320.29,325.31,330.34,340.42,350.49,360.58,370.67,380.77,390.88,400.98,411.12,421.26,431.43,441.61,451.80,462.02,472.24,482.49,492.74,503.02,513.32,523.63,533.98,544.35,554.74,565.17,575.59,586.04,596.52,607.02,617.53,628.07,638.63,649.22,659.84,670.47,681.14,691.82,702.52,713.27,724.04,734.82,745.62,756.44,767.29,778.18,789.11,800.03,810.99,821.95,843.98,866.08,888.27,910.56,932.93,955.38,977.92,1000.55,1023.25,1046.04,1068.89,1091.85,1114.86,1137.89,1161.07,1184.28,1207.57,1230.92,1254.34,1277.79,1301.31,1324.93,1348.55,1372.24,1395.97,1419.76,1443.60,1467.49,1491.44,1515.42,1539.44,1563.51,1587.63,1611.79,1635.97,1660.23,1684.51,1708.82,1733.17,1757.57,1782.00,1806.46,1830.96,1855.50,1880.10,1941.60,2003.30,2065.30,2127.40,2189.70,2252.10,2314.60,2377.40,2440.30,2503.20,2566.40};

const float Pressure[Number_of_values] = {
0.3363,0.3987,0.4690,0.5477,0.6355,0.7329,0.8405,0.9590,1.0889,1.1584,1.2311,1.3068,1.3860,1.4686,1.5546,1.6442,1.7375,1.8345,1.9352,2.149,2.379,2.626,2.892,3.176,3.481,3.806,4.153,4.522,4.915,5.332,5.775,6.245,6.742,7.268,7.824,8.411,9.031,9.684,10.37,11.10,11.86,12.66,13.50,14.38,15.31,16.28,17.30,18.36,19.84,20.64,21.86,23.13,24.46,25.85,27.29,28.80,30.38,32.02,33.72,35.50,37.35,39.27,41.31,43.35,45.55,47.75,52.59,57.60,63.09,68.98,75.29,82.05,89.28,97.00,105.2,114.0,123.4,133.3,143.9,155.2,167.1,179.7,193.1,207.2,222.2,238.0,254.7,272.3,290.8,310.4,330.9,352.5,375.3,399.1,424.2,450.5,478.0,506.9,537.1,568.8,601.9,636.5,672.8,710.5,750.0,791.2,834.1,878.9,925.6,974.2,1025,1161,1310,1475,1655,1852,2068,2303,2559,2837,3138,3464};

float interp(float x, const float *X, const float *Y) {
  x = constrain(x, X[0], X[Number_of_values-1]);
  for (int i = 1; i < Number_of_values; i++)
    if (x <= X[i])
      return Y[i-1] + (x-X[i-1])*(Y[i]-Y[i-1])/(X[i]-X[i-1]);
  return Y[Number_of_values-1];
}

float inv(float y, const float *Y, const float *X) {
  y = constrain(y, Y[0], Y[Number_of_values-1]);
  for (int i = 1; i < Number_of_values; i++)
    if (y <= Y[i])
      return X[i-1] + (y-Y[i-1])*(X[i]-X[i-1])/(Y[i]-Y[i-1]);
  return X[Number_of_values-1];
}

void setup() {
  pinMode(SWITCH, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(1000);
  delay(3000);
}

void loop() {
  map()
  float T0 = readPot(POT_T0, 216.65, 320.0);
  float P0 = readPot(POT_P0, 0.20, 1.10);
  float V0 = readPot(POT_V0, 0.0, 700.0);
  float m  = readPot(POT_M, 1.0, 80.0);

  float h0 = interp(T0, Temperature, Enthalpy);
  float pr0 = interp(T0, Temperature, Pressure);

  float h2 = h0 + V0*V0/2000.0;
  float T2 = inv(h2, H, Temperature);
  float pr2 = interp(T2, Temperature, Pressure);
  float P2 = P0*pr2/pr0;

  float P3 = Ratio_Pressure*P2;
  float pr3 = Ratio_Pressure*pr2;
  float T3 = inv(pr3, Pressure, Temperature);
  float h3 = interp(T3, Temperature, Enthalpy);

  float P4 = P3;
  float h4 = interp(T4, Temperature, Enthalpy);
  float pr4 = interp(T4, Temperature, Pressure);

  float h5 = h4-(h3-h2);
  float T5 = inv(h5, Enthalpy, Temperature);
  float pr5 = interp(T5, Temperature, Pressure);
  float P5 = P4*pr5/pr4;


  float P9 = P0;
  float pr9 = pr5*P9/P5;
  float T9 = inv(pr9, Pressure, Temperature);
  float h9 = interp(T9, Temperature, Enthalpy);
  float V9 = sqrt(max(0.0f, 2000.0f*(h5-h9)));
  float F = m*(V9-V0);

  bool valid = P5>P0 && pr3<=Pressure[N-1];
  bool run = digitalRead(SWITCH)==LOW && valid;
  esc.writeMicroseconds(run ? map((int)(m*10),10,800,1150,1800) : 1000);


  delay(250);
}