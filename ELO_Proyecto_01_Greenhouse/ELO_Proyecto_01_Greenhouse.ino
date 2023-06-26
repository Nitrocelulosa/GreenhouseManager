/* CODE BY MM
 * aprender: strncpy(destination , source, sizeof(destination)); // para pegar *char[] en *char[]
 *
 * 
 *  
 */
 
// Librerias usadas
#include <SoftwareSerial.h>
#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>
#include <DHT_U.h>
#include <EEPROM.h>
#include <Servo.h>

// PINES Digitales
const char bt_rx     =  2;
const char bt_tx     =  3;
const char pump      =  4;
const char fan       =  5;
const char servo_pin =  6;
const char t_sen_1   =  7;
const char t_sen_2   =  8;
const char LED01     =  9;
const char LED02     = 10;
const char LED03     = 11;
const char HEATER    = 12;
const char BUZZER    = 13;

// PINES Analogos (Equivalencias con el esquema)
//const char SoilMoisture = 14;  // A0
const char LUX_Sens     = 15;  // A1
const char ADC_2        = 16;  // A2
const char ADC_3        = 17;  // A3
const char ADC_6        = 20;  // A6
//const char ADC_7        = 21;  // A7
const char SoilMoisture = 21;  // A7 debido a un error de diseño en la palca

// Cosas constantes
const uint8_t ledOnState     = 200;
const uint8_t ledOffState    = 255;
const uint8_t openPossition  = 180;
const uint8_t closePossition =   0;

// Flags (Para evitar congestionar la ejecucion del codigo en runtime)
unsigned long lastMillState    = 0;
unsigned long lastSensorsCheck = 0;


// Enumeraciones para la simplicidad y expandibilidad del codigo
typedef enum{
  Lu, Ma, Mi, Ju, Vi, Sa, Do
} days;
typedef enum{
  Ene, Feb, Mar, Abr, May, Jun, Jul, Ago, Sep, Oct, Nob, Dic
} months;
typedef enum{
  Working, Failed, Compromized, Com_Err, Waiting, noCom, Sensing
} states;
typedef enum{
  ReturnToDefault, TotalReport, Report, Monitor, UpdateTime, Recived, RemoteControlActuator, QuitRemoteControlActuator
} comComands; // Comandos de comunicacion por bluetooth
typedef enum{
  MMRESET, MMSTART, MMSTOP, MMCHECK, MMGET, MMSET, MMTEST, MMDISCONECT  
} MMComands; // Los Comandos "MM" son semejantes a los comandos "AT" de los modems y al igual que estos la nomenclaura es "MM+" el comando o simplemente "MM" para pedir un reporte
typedef enum{
  LOWMOISTURE, HIGHMOISTURE, LOWTEMP, HIGHTEMP
} ControlStates;


// Estructurtas Utiles para la simplicidad y expandibilidad del codigo
struct Sensor{  // Sensor con valores enteros de 8bit sin signos
  char name[25];
  uint16_t id;
  uint8_t value;
  uint8_t min_val;
  uint8_t max_val;
  uint16_t storage_length;
  states state;
};
struct CommandMapping {   // Util para asignar funciones a un texto
  String command;
  void *function;
};

// Prototipado de funciones
void getSensorDataCommand(String command, Stream* port, uint16_t index);
void startSensorCommand(String command, Stream* port, uint16_t index);
void stopSensorCommand(String command, Stream* port, uint16_t index);
void getSensorListCommand(String command, Stream* port, uint16_t index);
void saveConfigurationCommand(String command, Stream* port, uint16_t index);
void loadConfigurationCommand(String command, Stream* port, uint16_t index);
void getEEPROMCommand(String command, Stream* port, uint16_t index);
void loadToEEPROMCommand(String command, Stream* port, uint16_t index);
void goAuto(String command, Stream* port, uint16_t index);
void goManual(String command, Stream* port, uint16_t index);
void OpenWindow();
void CloseWindow();
void onFan();
void offFan();
void checkMoisture();
void checkTemp();

// Arreglo de comandos
CommandMapping MMCommandMap[] = {
  {"MM+GET_SENSOR_DATA"   , &getSensorDataCommand    },
  {"MM+START_SENSOR"      , &startSensorCommand      },
  {"MM+STOP_SENSOR"       , &stopSensorCommand       },
  {"MM+GET_SENSOR_LIST"   , &getSensorListCommand    },
  {"MM+SAVE_CONFIGURATION", &saveConfigurationCommand},
  {"MM+LOAD_CONFIGURATION", &loadConfigurationCommand},
  {"MM+GET_EEPROM"        , &getEEPROMCommand        },
  {"MM+LOAD_TO_EEPROM"    , &loadToEEPROMCommand     },
  {"MM+OPEN_WINDOW"       , &OpenWindow              },
  {"MM+CLOSE_WINDOW"      , &CloseWindow             },
  {"MM+ON_FAN"            , &onFan                   },
  {"MM+OFF_FAN"           , &offFan                  },
  {"MM+CHECK_MOISTURE"    , &checkMoisture           },
  {"MM+CHECK_TEMP"        , &checkTemp               },
  {"MM+AUTO"              , &goAuto                  },
  {"MM+MANUAL"            , &goManual                }
};

// Arreglo de comandos más robusto
/*CommandMapping MMCommandMap[] = {
  {"MM+GET_SENSOR_DATA"   , reinterpret_cast<void *>(&getSensorDataCommand)    },
  {"MM+START_SENSOR"      , reinterpret_cast<void *>(&startSensorCommand)      },
  {"MM+STOP_SENSOR"       , reinterpret_cast<void *>(&stopSensorCommand)       },
  {"MM+GET_SENSOR_LIST"   , reinterpret_cast<void *>(&getSensorListCommand)    },
  {"MM+SAVE_CONFIGURATION", reinterpret_cast<void *>(&saveConfigurationCommand)},
  {"MM+LOAD_CONFIGURATION", reinterpret_cast<void *>(&loadConfigurationCommand)},
};*/


// Declaracion de los Modulos y Sensores
SoftwareSerial bluetooth(bt_tx, bt_rx); //rx, tx
DHT dht_1(t_sen_1,DHT11);
DHT dht_2(t_sen_2,DHT11);
DateTime now;
Servo servo;
//DS1307 RTC;
//RTC_DS3231 rtc;


//                     Los Valores de acá  no son relevantes aparte de los dos primeros y el último
Sensor temp_inside   = {"Temperatura Interna",0,0,0,0,2};
Sensor temp_outside  = {"Temperatura Externa",1,0,0,0,2};
Sensor hum_inside    = {"Humedad Interna"    ,2,0,0,0,2};
Sensor hum_outside   = {"Humedad Externa"    ,3,0,0,0,2};
Sensor soil_moisture = {"Humedad Suelo"      ,4,0,0,0,2};

Sensor* sensorsList[] = {
  &temp_inside  , 
  &temp_outside ,
  &hum_inside   , 
  &hum_outside  ,
  &soil_moisture
};

states state = Waiting;
bool isAuto = true;


void setup() {
  // Inicializacion de los modulos
  Wire.begin();
  bluetooth.begin(9600);
  Serial.begin(57600);
  dht_1.begin();
  dht_2.begin();
  servo.attach(servo_pin);

  // Se declaran como salidas/entradas digitales los siguientes pines
  pinMode(pump  , 1);
  pinMode(fan   , 1);
  pinMode(LED01 , 1);
  pinMode(LED02 , 1);
  pinMode(LED03 , 1);
  pinMode(HEATER, 1);
  pinMode(BUZZER, 1);
  digitalWrite(LED01,1);
  digitalWrite(LED02,1);
  digitalWrite(LED03,0);

  //rtc.begin();
  digitalWrite(LED01,0);
  digitalWrite(LED02,1);
  digitalWrite(LED03,1);

  SensorDataCheckEEPROM(&temp_inside  );
  SensorDataCheckEEPROM(&temp_outside );
  SensorDataCheckEEPROM(&hum_inside   );
  SensorDataCheckEEPROM(&hum_outside  );
  SensorDataCheckEEPROM(&soil_moisture);  

  //PrintAllEEPROM(&Serial);
  Serial.println();
  Serial.println();
  Serial.println();

  bluetooth.setTimeout(100);
  Serial.setTimeout(100);
  //now = rtc.now();
  //delay(2000);

}

void loop() {
  if(millis()-lastSensorsCheck > 2000){
    //ShowState(Sensing);
    //makeReport();
    checkMoisture();
    checkTemp();
    lastSensorsCheck = millis();
    //digitalWrite(pump, 1);
    //digitalWrite(HEATER, 0);
    //bluetooth.println("Vamo");

  }
  //digitalWrite(pump, 0);
  //digitalWrite(HEATER, 1);
  
  //Serial.println(EEPROM.length());
  checkComm(&Serial);
  checkComm(&bluetooth);

  ShowState(state);
}

void checkComm(Stream* port){
  String inComm = "";
  if(port->available()){
    inComm = port->readStringUntil("\n");
    inComm.trim();
    inComm.toUpperCase();
    if(inComm.equalsIgnoreCase("MM")){
      // Reporte Completo
      port->println("Reporte Completo en Camino!");
      beep(25);
    }else if(inComm.startsWith("MM+")){
      processCommand(inComm, port);
    }else{
      port->println("Comando Invalido");
      beep(25);
    }
  }
}

void beep(uint16_t t){   // Se encarga de hacer "BEEP" y era 
  digitalWrite(BUZZER, 1);
  delay(t);  
  digitalWrite(BUZZER, 0);
}

// Mostrar estado actual
void ShowState(states state){
  uint8_t r = ledOffState;
  uint8_t g = ledOffState;
  uint8_t b = ledOffState;
  switch(state){
    case Working:
      g = ledOnState;
      break;    
    case Failed:
      r = ledOnState;
      break;    
    case Compromized:
      if(millis()-lastMillState >= 500){
        r = ledOnState;
      }else{
        r = ledOffState;
      }
      break;    
    case Com_Err: // Error en la comunicacion 
      if(millis()-lastMillState >= 750){
        r = ledOnState; b = ledOffState;
      }else if(millis()-lastMillState >= 500){
        r = ledOffState; b = ledOnState;        
      }else{
        r = ledOffState; b = ledOffState;
      }
      break;
    case Waiting: // Espera de Escritura/Lectura
      if(millis()-lastMillState >= 500){
        g = ledOnState;
      }else{
        g = ledOffState;
      }
      break;    
    case noCom: // Sin Comunicaciones
      if(millis()-lastMillState >= 500){
        b = ledOnState;
      }else{
        b = ledOffState;
      }
      break;
    case Sensing: // Mientras esta sensando
      r = ledOnState;
      g = ledOnState;
      break;        
  }
  analogWrite(LED01,r);
  analogWrite(LED02,g);
  analogWrite(LED03,b);
  if(millis()-lastMillState >= 1000){
    lastMillState = millis();
  }
}

void SensorDataCheckEEPROM(Sensor *sensor){
  uint16_t offset = 128;
  if(sensor->id > 127) return;
  uint8_t id_content = EEPROM.read(sensor->id);
  for(uint16_t i = 0; i<sensor->id; i++){
    offset = offset + EEPROM.read(i);      
  }   
  if(id_content == 0 || id_content > 32){
    if(sensor->storage_length == 0 || sensor->storage_length > 32) return;
    EEPROM.update(sensor->id, sensor->storage_length);
    EEPROM.update(offset    , sensor->min_val       );
    EEPROM.update(offset+1  , sensor->max_val       );
  }else{     
    sensor->min_val = EEPROM.read(offset  );
    sensor->max_val = EEPROM.read(offset+1);
  }
}
void SensorDataUpdateEEPROM(Sensor *sensor){
  uint16_t offset = 128;
  if(sensor->id > 127) return;
  for(uint16_t i = 0; i<sensor->id; i++){
    offset = offset + EEPROM.read(i);      
  }   
  if(sensor->storage_length == 0 || sensor->storage_length > 32) return;
  EEPROM.update(sensor->id, sensor->storage_length);
  EEPROM.update(offset    , sensor->min_val       );
  EEPROM.update(offset+1  , sensor->max_val       );
}
void PrintAllEEPROM(Stream* port){
  String line = "";
  for(uint16_t i = 0; i < EEPROM.length()/16; i++){
    for(uint16_t j = 0; j < 16; j++){
      uint8_t value = EEPROM.read(i*16+j);
      line = line + String(value,HEX) + " ";
    }
    port->println(line);
    line = "";
    if((i+1)%8 == 0)  port->println();
  }
}


/*
void processCommand(String command, Stream* port) {
  for (int i = 0; i < sizeof(MMCommandMap) / 2; i++) {
    if (command.equals(MMCommandMap[i].command)) {
      void (*functionPtr)(String,Stream*);
      functionPtr = reinterpret_cast<void (*)(Stream*)>(MMCommandMap[i].function); // No se que chucha pero funcionó
      functionPtr(command,port);
      return;
    }
  }
  
  // Si no se encuentra una coincidencia
  port->println("Comando no válido");
}*/


void processCommand(String command, Stream* port) {
  uint16_t index = 0;
  for (int i = 0; i < sizeof(MMCommandMap)/sizeof(CommandMapping); i++) {
    if (command.startsWith(MMCommandMap[i].command)) {
      index = MMCommandMap[i].command.length();
      // un poco de magia negra para poder invocar la funcion a la que apuntaba el puntero
      void (*functionPtr)(String, Stream*,uint16_t) = reinterpret_cast<void (*)(String, Stream*,uint16_t)>(MMCommandMap[i].function);
      functionPtr(command, port, index);
      return;
    }
  }
  
  // Si no se encuentra una coincidencia, esque eres bruto
  port->println("Comando no válido");
}



void getSensorDataCommand(String command, Stream* port, uint16_t index){
  
}
void startSensorCommand(String command, Stream* port, uint16_t index){

}
void stopSensorCommand(String command, Stream* port, uint16_t index){
  
}
void getSensorListCommand(String command, Stream* port, uint16_t index){
  port->println("Lista Sensosres [Id] [Nombre]");
  for(uint16_t i = 0; i < sizeof(sensorsList)/sizeof(Sensor*); i++){
    port->print((String)(sensorsList[i]->id) + " - ");
    port->print(sensorsList[i]->name);
    port->println();
  }
}
void saveConfigurationCommand(String command, Stream* port, uint16_t index){
  
}
void loadConfigurationCommand(String command, Stream* port, uint16_t index){
  
}
void getEEPROMCommand(String command, Stream* port, uint16_t index){
  PrintAllEEPROM(port);
}
void loadToEEPROMCommand(String command, Stream* port, uint16_t index){
  
}

void goAuto(String command, Stream* port, uint16_t index){
  if(isAuto ) port->println("Ya estaba en Automatico!");
  if(!isAuto) port->println("Cambiado a Automatico!"  );
  isAuto = true;
  beep(125);
  state = Working;
}
void goManual(String command, Stream* port, uint16_t index){
  if(!isAuto) port->println("Ya estaba en Manual!");
  if(isAuto ) port->println("Cambiado a Manual!"  );
  isAuto = false;
  beep(125);
  state = Waiting;
}






// Utilidades
void OpenWindow(){
  servo.write(openPossition);
}
void CloseWindow(){
  servo.write(closePossition);
}
void checkMoisture(){
  soil_moisture.value = map(analogRead(SoilMoisture),0,1023,0,255);
}
void checkTemp(){
  uint8_t ti = 0;
  uint8_t to = 0;

  float ti_f = dht_1.readTemperature();
  float to_f = dht_2.readTemperature();

  if(isnan(ti_f)){ // En el caso que el resultado leido no se aun numero (NAN) es que hay error
    bluetooth.println("Error en Medicion de " + (String)temp_inside.name + "; Resultado: NAN");
    Serial.println(   "Error en Medicion de " + (String)temp_inside.name + "; Resultado: NAN");
    temp_inside.state = Failed;
  }else{
    ti = map(ti_f,0,50,0,255);
    temp_inside.state = Working;
    Serial.println(ti);
  }

  if(isnan(to_f)){ // En el caso que el resultado leido no se aun numero (NAN) es que hay error
    bluetooth.println("Error en Medicion de " + (String)temp_outside.name + "; Resultado: NAN");
    Serial.println(   "Error en Medicion de " + (String)temp_outside.name + "; Resultado: NAN");
    temp_outside.state = Failed;
  }else{
    to = map(to_f,0,50,0,255);
    temp_outside.state = Working;
    Serial.println(to);
  }

  temp_inside.value  = ti;
  temp_outside.value = to;
}
void onFan(){
  beep(500);
  digitalWrite(fan,1);
}
void offFan(){
  beep(125);
  digitalWrite(fan,0);
}


void Heater(bool is_on){}



 






/*

Por si los uso

void enviarValorSerial(Stream* serialPtr, int valor) {
  serialPtr->println(valor);
}

*/





