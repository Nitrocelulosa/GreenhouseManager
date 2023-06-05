/* CODE BY MM
 * aprender: strncpy(destination , source, sizeof(destination)); // para pegar *char[] en *char[]
 *
 *  
 */
// Librerias usadas
#include <SoftwareSerial.h>
#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>
#include <DHT_U.h>

// PINES Digitales
const char bt_rx   =  2;
const char bt_tx   =  3;
const char pump    =  4;
const char fan     =  5;
const char servo   =  6;
const char t_sen_1 =  7;
const char t_sen_2 =  8;
const char LED01   =  9;
const char LED02   = 10;
const char LED03   = 11;
const char HEATER  = 12;
const char BUZZER  = 13;

// PINES Analogos (Equivalencias con el esquema)
const char SoilMoisture = 14;  // A0
const char LUX_Sens     = 15;  // A1
const char ADC_2        = 16;  // A2
const char ADC_3        = 17;  // A3
const char ADC_6        = 20;  // A6
const char ADC_7        = 21;  // A7

// Enumeraciones para la simplicidad y expandibilidad del codigo
typedef enum{
  Lu, Ma, Mi, Ju, Vi, Sa, Do
} days;
typedef enum{
  Ene, Feb, Mar, Abr, May, Jun, Jul, Ago, Sep, Oct, Nob, Dic
} months;
typedef enum{
  Working, Failed, Compromized, Com_Err, Waiting, noCom
} states;
typedef enum{
  ReturnToDefault, TotalReport, Report, Monitor, UpdateTime, Recived, RemoteControlActuator, QuitRemoteControlActuator
} comComands;

// Estructurtas Utiles para la simplicidad y expandibilidad del codigo
struct Sensor_int{  // Sensor con valores enteros de 16bit sin signos
  char name[26];
  uint16_t value;
  uint16_t min_val;
  uint16_t max_val;
}; // Struct Size: 32 bytes

struct Sensor_float{  // Sensor con valores flotantes
  char name[20];
  float value;
  float min_val;
  float max_val;
}; // Struct Size: 32 bytes

// Declaracion de los Modulos y Sensores
SoftwareSerial bluetooth(bt_tx, bt_rx); //rx, tx
DHT dht_1(t_sen_1,DHT11);
DHT dht_2(t_sen_2,DHT11);
RTC_DS1307 RTC;


void setup() {
  // Se declaran como salidas/entradas digitales los siguientes pines
  pinMode(pump  , 1);
  pinMode(fan   , 1);
  pinMode(servo , 1);
  pinMode(LED01 , 1);
  pinMode(LED02 , 1);
  pinMode(LED03 , 1);
  pinMode(HEATER, 1);
  pinMode(BUZZER, 1);

  // Inicializacion de los modulos
  bluetooth.begin(9600);
  Serial.begin(9600);
  dht_1.begin();
  dht_2.begin();
  Wire.begin();
  RTC.begin();

  bluetooth.setTimeout(100);
  Serial.setTimeout(100);
  DateTime now = RTC.now();
  
}

void loop() {
  makeReport();
  delay(2000);
}

void beep(uint16_t t){   // Se encarga de hacer "BEEP" y era 
  digitalWrite(BUZZER, 1);
  delay(t);  
  digitalWrite(BUZZER, 0);
}

void makeReport(){
  Serial.println("Reporte Completo");
  makeTempReport();
  makeHumReport();
  Serial.println();
}

void makeTempReport(){
  Serial.println("Reporte Temperatura:");
  Serial.println("- Temperaturas (Interna/Externa):");
  float t1 = dht_1.readTemperature();
  float t2 = dht_2.readTemperature();
  Serial.print((String)t1 + " - " + (String)t2);
  if(isnan(t1) || isnan(t2)){
    Serial.println();
    Serial.print("Error(es) en ");
    if(isnan(t1)) Serial.print("Sensor Temperatura Interna   ");
    if(isnan(t2)) Serial.print("Sensor Temperatura Externa   ");
    Serial.println();
  }
  Serial.println();
}
void makeHumReport(){
  Serial.println("Reporte Humedad:");
  Serial.println("- Humedad (Interna/Externa):");
  float h1 = dht_1.readHumidity();
  float h2 = dht_2.readHumidity();
  Serial.print((String)h1 + " - " + (String)h2);
  if(isnan(h1) || isnan(h2)){
    Serial.println();
    Serial.print("Error en ");
    if(isnan(h1)) Serial.print("Sensor Humedad Interna   ");
    if(isnan(h2)) Serial.print("Sensor Humedad Externa   ");
  }
  Serial.println();
}
