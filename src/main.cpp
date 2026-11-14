#include <Arduino.h>
#include <ESP32Servo.h>

#define POT_T0 36
#define T0_min 216.65
#define T0_max 320.0

#define POT_P0 39
#define P0_min 0.20
#define P0_max 1.10

#define POT_V0 34
#define V0_min 0.0
#define V0_max 700.0

#define POT_M  35
#define M_min 0.0 //Ajustar flujo masico, buscar en linea 
#define M_max 77.0

#define ON_OF_SWITCH 26

#define ESC_PIN 27
#define ESC_Hertz 50
#define ESC_stop 1000
#define ESC_max 2000

#define Baudreate 115200

#define Ratio_Pressure 13.5
#define Temperature_4 1265.0
#define Number_of_values 122

#define Initial_wait 1000
#define ADC_resolution_float 4095.0
#define UNIT_compensation 2000.0
#define ON "ON"
#define OFF "OFF"
#define All_zeros 0

#define Speed_up_time 4000
#define Speed_up_levels 10
#define First_speed_up_level 0

#define FIRST_LED_CC_PIN 32
#define SECOND_LED_CC_PIN 33
#define LED_LUB_PIN 14
#define LED_AIR_PIN 13

const float Temperature[Number_of_values] = {
200,210,220,230,240,250,260,270,280,285,290,295,300,305,310,315,320,325,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510,520,530,540,550,560,570,580,590,600,610,620,630,640,650,660,670,680,690,700,710,720,730,740,750,760,770,780,790,800,820,840,860,880,900,920,940,960,980,1000,1020,1040,1060,1080,1100,1120,1140,1160,1180,1200,1220,1240,1260,1280,1300,1320,1340,1360,1380,1400,1420,1440,1460,1480,1500,1520,1540,1560,1580,1600,1620,1640,1660,1680,1700,1750,1800,1850,1900,1950,2000,2050,2100,2150,2200,2250};

const float Enthalpy[Number_of_values] = {
199.97,209.97,219.97,230.02,240.02,250.05,260.09,270.11,280.13,285.14,290.16,295.17,300.19,305.22,310.24,315.27,320.29,325.31,330.34,340.42,350.49,360.58,370.67,380.77,390.88,400.98,411.12,421.26,431.43,441.61,451.80,462.02,472.24,482.49,492.74,503.02,513.32,523.63,533.98,544.35,554.74,565.17,575.59,586.04,596.52,607.02,617.53,628.07,638.63,649.22,659.84,670.47,681.14,691.82,702.52,713.27,724.04,734.82,745.62,756.44,767.29,778.18,789.11,800.03,810.99,821.95,843.98,866.08,888.27,910.56,932.93,955.38,977.92,1000.55,1023.25,1046.04,1068.89,1091.85,1114.86,1137.89,1161.07,1184.28,1207.57,1230.92,1254.34,1277.79,1301.31,1324.93,1348.55,1372.24,1395.97,1419.76,1443.60,1467.49,1491.44,1515.42,1539.44,1563.51,1587.63,1611.79,1635.97,1660.23,1684.51,1708.82,1733.17,1757.57,1782.00,1806.46,1830.96,1855.50,1880.10,1941.60,2003.30,2065.30,2127.40,2189.70,2252.10,2314.60,2377.40,2440.30,2503.20,2566.40};

const float Pressure[Number_of_values] = {
0.3363,0.3987,0.4690,0.5477,0.6355,0.7329,0.8405,0.9590,1.0889,1.1584,1.2311,1.3068,1.3860,1.4686,1.5546,1.6442,1.7375,1.8345,1.9352,2.149,2.379,2.626,2.892,3.176,3.481,3.806,4.153,4.522,4.915,5.332,5.775,6.245,6.742,7.268,7.824,8.411,9.031,9.684,10.37,11.10,11.86,12.66,13.50,14.38,15.31,16.28,17.30,18.36,19.84,20.64,21.86,23.13,24.46,25.85,27.29,28.80,30.38,32.02,33.72,35.50,37.35,39.27,41.31,43.35,45.55,47.75,52.59,57.60,63.09,68.98,75.29,82.05,89.28,97.00,105.2,114.0,123.4,133.3,143.9,155.2,167.1,179.7,193.1,207.2,222.2,238.0,254.7,272.3,290.8,310.4,330.9,352.5,375.3,399.1,424.2,450.5,478.0,506.9,537.1,568.8,601.9,636.5,672.8,710.5,750.0,791.2,834.1,878.9,925.6,974.2,1025,1161,1310,1475,1655,1852,2068,2303,2559,2837,3138,3464};

float interp(float x, const float *X, const float *Y) {
  for (int i = 1; i < Number_of_values; i++)
    if (x <= X[i])
      return Y[i-1] + (x-X[i-1])*(Y[i]-Y[i-1])/(X[i]-X[i-1]);
  return Y[Number_of_values-1];
}

float inv(float y, const float *Y, const float *X) {
  for (int i = 1; i < Number_of_values; i++)
    if (y <= Y[i])
      return X[i-1] + (y-Y[i-1])*(X[i]-X[i-1])/(Y[i]-Y[i-1]);
  return X[Number_of_values-1];
}

float readPot(int pin, float minVal, float maxVal) {
  return minVal + analogRead(pin)*(maxVal-minVal)/ADC_resolution_float;
}

typedef enum Motor_states
{
  Apagado = 0, Arrancando, Encendido
}Motor_states;

String states_names (Motor_states state)
{
  switch(state)
  {
    case Apagado:
      return "Apagado";

    case Arrancando:
      return "Arrancando";

    case Encendido:
      return "Encendido";
  }
}

struct CC
{
  String CC_State = OFF;
  bool CC_LED = LOW;
};

struct LUB
{
  String LUB_State = OFF;
  bool LUB_LED = LOW;
};

struct AIR
{
  String AIR_State = OFF;
  bool AIR_LED = LOW;
};


struct Display
{
  float T0;
  float P0; 
  float V0;
  float m;
  float T2;
  float P2;
  float T3;
  float P3;
  float T4 = 1265.0;
  float P4;
  float T5;
  float P5;
  float T9;
  float P9;  
  float V9;
  float F;
  float V_N1 = 0;
  CC ledCC;
  LUB ledLub;
  AIR ledAir;
}; 

void send_telemetry(Motor_states state, Display output)
{
  String state_name = states_names(state);

  Serial.print(state_name);
  
  Serial.print(','); Serial.print(output.T0, 2);
  Serial.print(','); Serial.print(output.P0, 2);
  Serial.print(','); Serial.print(output.V0, 2);
  Serial.print(','); Serial.print(output.m, 2);

  Serial.print(','); Serial.print(output.T2, 2);
  Serial.print(','); Serial.print(output.P2, 2);

  Serial.print(','); Serial.print(output.T3, 2);
  Serial.print(','); Serial.print(output.P3, 2);

  Serial.print(','); Serial.print(output.T4, 2);
  Serial.print(','); Serial.print(output.P4, 2);

  Serial.print(','); Serial.print(output.T5, 2);
  Serial.print(','); Serial.print(output.P5, 2);

  Serial.print(','); Serial.print(output.T9, 2);
  Serial.print(','); Serial.print(output.P9, 2);
  Serial.print(','); Serial.print(output.V9, 2);
  Serial.print(','); Serial.print(output.F, 2);
  Serial.print(','); Serial.print(output.V_N1, 2);

  Serial.print(','); Serial.print(output.ledCC.CC_State);
  Serial.print(','); Serial.print(output.ledCC.CC_LED);
  Serial.print(','); Serial.print(output.ledLub.LUB_State);
  Serial.print(','); Serial.print(output.ledLub.LUB_LED);
  Serial.print(','); Serial.print(output.ledAir.AIR_State);
  Serial.print(','); Serial.println(output.ledAir.AIR_LED);
}

void send_state(Motor_states state, float number, Display output)
{
  String state_name = states_names(state);

  Serial.print(state_name);

  Serial.print(','); Serial.print(number, 2);
  Serial.print(','); Serial.print(number, 2);
  Serial.print(','); Serial.print(number, 2);
  Serial.print(','); Serial.print(number, 2);

  Serial.print(','); Serial.print(number, 2);
  Serial.print(','); Serial.print(number, 2);

  Serial.print(','); Serial.print(number, 2);
  Serial.print(','); Serial.print(number, 2);

  Serial.print(','); Serial.print(number, 2);
  Serial.print(','); Serial.print(number, 2);

  Serial.print(','); Serial.print(number, 2);
  Serial.print(','); Serial.print(number, 2);

  Serial.print(','); Serial.print(number, 2);
  Serial.print(','); Serial.print(number, 2);
  Serial.print(','); Serial.print(number, 2);
  Serial.print(','); Serial.print(number, 2);
  Serial.print(','); Serial.print(output.V_N1, 2);

  Serial.print(','); Serial.print(output.ledCC.CC_State);
  Serial.print(','); Serial.print(output.ledCC.CC_LED);
  Serial.print(','); Serial.print(output.ledLub.LUB_State);
  Serial.print(','); Serial.print(output.ledLub.LUB_LED);
  Serial.print(','); Serial.print(output.ledAir.AIR_State);
  Serial.print(','); Serial.println(output.ledAir.AIR_LED);
  
}

void calculations(Display &output)
{
  output.T0 = readPot(POT_T0, T0_min, T0_max);
  output.P0 = readPot(POT_P0, P0_min, P0_max);
  output.V0 = readPot(POT_V0, V0_min, V0_max);
  output.m  = readPot(POT_M, M_min, M_max);

  float h0 = interp((float)output.T0, Temperature, Enthalpy);
  float pr0 = interp((float)output.T0, Temperature, Pressure);

  float h2 = h0 + output.V0*output.V0/UNIT_compensation;
  output.T2 = inv(h2, Enthalpy, Temperature);
  float pr2 = interp(output.T2, Temperature, Pressure);
  output.P2 = output.P0*pr2/pr0;

  output.P3 = Ratio_Pressure*output.P2;
  float pr3 = Ratio_Pressure*pr2;
  output.T3 = inv(pr3, Pressure, Temperature);
  float h3 = interp((float)output.T3, Temperature, Enthalpy);

  output.P4 = output.P3;
  float h4 = interp(Temperature_4, Temperature, Enthalpy);
  float pr4 = interp(Temperature_4, Temperature, Pressure);

  float h5 = h4-(h3-h2);
  output.T5 = inv(h5, Enthalpy, Temperature);
  float pr5 = interp((float)output.T5, Temperature, Pressure);
  output.P5 = output.P4*pr5/pr4;

  output.P9 = output.P0;
  float pr9 = pr5*output.P9/output.P5;
  output.T9 = inv(pr9, Pressure, Temperature);
  float h9 = interp((float)output.T9, Temperature, Enthalpy);
  output.V9 = sqrt(max(0.0f, 2000.0f*(h5-h9)));
  output.F = output.m*(output.V9-output.V0);
}

bool speed_up_motor (Motor_states state, Servo &esc, unsigned long &lasttime, Display &output)
{

  static int i = First_speed_up_level;

  if (state == Apagado)
  {
    i = First_speed_up_level;
  }

  esc.writeMicroseconds(map(i, First_speed_up_level, Speed_up_levels, ESC_stop, ESC_max));
  output.V_N1 = i*10;
    
  if(i == Speed_up_levels)
  {
    i = First_speed_up_level;
    return true;
  }
  else
  {
    i++;
    lasttime = millis();
    return false;
  }
}



String receive()
{
  String command;
  if(Serial.available())
  {
    while(Serial.available())
    {
      command = Serial.read();
    }
  }
  else
  {
    command = "NULL";
  }
  return command;
}


void commands(String command, Display &output)
{
 if (command == "LUB")
  {
    output.ledLub.LUB_State = !output.ledLub.LUB_State;
    output.ledLub.LUB_LED = !output.ledLub.LUB_LED;
    digitalWrite(LED_LUB_PIN, output.ledLub.LUB_LED);
    return;
  }
  else if (command == "AIR")
  {
    output.ledAir.AIR_State = !output.ledAir.AIR_State;
    output.ledAir.AIR_LED = !output.ledAir.AIR_LED;
    digitalWrite(LED_AIR_PIN, output.ledAir.AIR_LED);
    return;
  }
  else
  {
    return;
  }
}

Servo esc;
Motor_states state = Apagado;
Display output;
unsigned long last_time = 0;
bool motor_fully_on = false;

void setup() {

  Serial.begin(Baudreate);
  pinMode(ON_OF_SWITCH, INPUT_PULLDOWN);
  pinMode(FIRST_LED_CC_PIN, OUTPUT);
  pinMode(SECOND_LED_CC_PIN, OUTPUT);
  esc.setPeriodHertz(ESC_Hertz);
  esc.attach(ESC_PIN, ESC_stop, ESC_max);
  esc.writeMicroseconds(ESC_stop);
  send_state(state, All_zeros, output);
  state = Apagado;
  output.V_N1 = 0;
  esc.writeMicroseconds(ESC_stop);
  output.ledCC.CC_State = OFF;
  output.ledCC.CC_LED = LOW;
  digitalWrite(FIRST_LED_CC_PIN, output.ledCC.CC_LED);
  digitalWrite(SECOND_LED_CC_PIN, output.ledCC.CC_LED);
}

void loop() {

  bool run = digitalRead(ON_OF_SWITCH);
  
  switch(state)
  {
    case Apagado:
      if(!run)
      {
        send_state(state, All_zeros, output);
        motor_fully_on = false;
      }
      else
      {
        motor_fully_on = speed_up_motor(state, esc, last_time, output);
        state = Arrancando;
        last_time = millis(); //CHECAR
      }
    break;

    case Arrancando:

      if(run)
      {
        if(!motor_fully_on)
        {
          if(millis() - last_time >= Speed_up_time/Speed_up_levels)
          {
            motor_fully_on = speed_up_motor(state, esc, last_time, output);
            send_state(state, All_zeros, output);
          }

          return;
        }
        else
        {
          state = Encendido;
          output.ledCC.CC_State = ON;
          output.ledCC.CC_LED = HIGH;
          digitalWrite(FIRST_LED_CC_PIN, output.ledCC.CC_LED);
          digitalWrite(SECOND_LED_CC_PIN, output.ledCC.CC_LED);
        }
      }
      else
      {
        state = Apagado;
        output.V_N1 = 0;
        esc.writeMicroseconds(ESC_stop);
        output.ledCC.CC_State = OFF;
        output.ledCC.CC_LED = LOW;
        digitalWrite(FIRST_LED_CC_PIN, output.ledCC.CC_LED);
        digitalWrite(SECOND_LED_CC_PIN, output.ledCC.CC_LED);
      }
    break;

    case Encendido:
    if(run)
    {
      calculations(output);
      send_telemetry(state, output);
    }
    else
    {
        state = Apagado;
        output.V_N1 = 0;
        esc.writeMicroseconds(ESC_stop);
        output.ledCC.CC_State = OFF;
        output.ledCC.CC_LED = LOW;
        digitalWrite(FIRST_LED_CC_PIN, output.ledCC.CC_LED);
        digitalWrite(SECOND_LED_CC_PIN, output.ledCC.CC_LED);
    }
    break;
  }

  delay(250);

}