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

// Flags (Para evitar congestionar la ejecucion del codigo en runtime)
unsigned long lastMillState = 0;
unsigned long lastReportMillis = 0;


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
};
struct CommandMapping {   // Util para asignar funciones a un texto
  String command;
  void *function;
};

// Prototipado de funciones
void getSensorDataCommand(String command, Stream* port);
void startSensorCommand(String command, Stream* port);
void stopSensorCommand(String command, Stream* port);
void getSensorListCommand(String command, Stream* port);
void saveConfigurationCommand(String command, Stream* port);
void loadConfigurationCommand(String command, Stream* port);


// Arreglo de comandos

CommandMapping MMCommandMap[] = {
  {"MM+GET_SENSOR_DATA"   , &getSensorDataCommand    },
  {"MM+START_SENSOR"      , &startSensorCommand      },
  {"MM+STOP_SENSOR"       , &stopSensorCommand       },
  {"MM+GET_SENSOR_LIST"   , &getSensorListCommand    },
  {"MM+SAVE_CONFIGURATION", &saveConfigurationCommand},
  {"MM+LOAD_CONFIGURATION", &loadConfigurationCommand},
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
RTC_DS1307 RTC;
DateTime now;


//                     Los Valores de acá  no son relevantes aparte de los dos primeros y el último
Sensor temp_inside   = {"Temperatura Interna",0,0,0,0,2};
Sensor temp_outside  = {"Temperatura Externa",1,0,0,0,2};
Sensor hum_inside    = {"Humedad Interna"    ,2,0,0,0,2};
Sensor hum_outside   = {"Humedad Externa"    ,3,0,0,0,2};
Sensor soil_moisture = {"Humedad Suelo"      ,4,0,0,0,2};

int i = 0;

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

  SensorDataCheckEEPROM(&temp_inside  );
  SensorDataCheckEEPROM(&temp_outside );
  SensorDataCheckEEPROM(&hum_inside   );
  SensorDataCheckEEPROM(&hum_outside  );
  SensorDataCheckEEPROM(&soil_moisture);  

  PrintAllEEPROM();
  Serial.println();
  Serial.println();
  Serial.println();

  bluetooth.setTimeout(100);
  Serial.setTimeout(100);
  now = RTC.now();
  delay(20000);

}

void loop() {
  /*
  now = RTC.now();
  if(millis()-lastReportMillis > 10000){
    makeReport();
    lastReportMillis = millis();
    i++;
  }
  if(i>5) i = 0;
  ShowState(i);
  //Serial.println(EEPROM.length());
  */
}

void beep(uint16_t t){   // Se encarga de hacer "BEEP" y era 
  digitalWrite(BUZZER, 1);
  delay(t);  
  digitalWrite(BUZZER, 0);
}

// Mostrar estado actual
void ShowState(states state){
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  switch(state){
    case Working:
      g = 1;
      break;    
    case Failed:
      r = 1;
      break;    
    case Compromized:
      if(millis()-lastMillState >= 500){
        r = 1;
      }else{
        r = 0;
      }
      break;    
    case Com_Err: // Error en la comunicacion 
      if(millis()-lastMillState >= 750){
        r = 1; b = 0;
      }else if(millis()-lastMillState >= 500){
        r = 1; b = 1;        
      }else{
        r = 0; b = 0;
      }
      break;
    case Waiting: // Espera de Escritura/Lectura
      if(millis()-lastMillState >= 500){
        g = 1;
      }else{
        g = 0;
      }
      break;    
    case noCom: // Sin Comunicaciones
      if(millis()-lastMillState >= 500){
        b = 1;
      }else{
        b = 0;
      }
      break;        
  }
  digitalWrite(LED01,r);
  digitalWrite(LED02,g);
  digitalWrite(LED03,b);
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
void PrintAllEEPROM(){
  String line = "";
  for(uint16_t i = 0; i < EEPROM.length()/16; i++){
    for(uint16_t j = 0; j < 16; j++){
      uint8_t value = EEPROM.read(i*16+j);
      line = line + "0x" + String(value,HEX) + " ";
    }
    Serial.println(line);
    line = "";
    if((i+1)%8 == 0)  Serial.println();
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
  
  // Si no se encuentra una coincidencia, esque eres bruto
  port->println("Comando no válido");
}*/


void processCommand(String command, Stream* port) {
  for (int i = 0; i < sizeof(MMCommandMap) / 2; i++) {
    if (command.equalsignorecase(MMCommandMap[i].command)) {
      void (*functionPtr)(String, Stream*) = reinterpret_cast<void (*)(String, Stream*)>(MMCommandMap[i].function);
      functionPtr(command, port);
      return;
    }
  }
  
  // Si no se encuentra una coincidencia
  port->println("Comando no válido");
}



void getSensorDataCommand(String command, Stream* port){
  
}
void startSensorCommand(String command, Stream* port){

}
void stopSensorCommand(String command, Stream* port){
  
}
void getSensorListCommand(String command, Stream* port){

}
void saveConfigurationCommand(String command, Stream* port){
  
}
void loadConfigurationCommand(String command, Stream* port){
  
}



// Utilidades
void OpenWindow(){}
void Heater(bool is_on){}










/*

Por si los uso

void enviarValorSerial(Stream* serialPtr, int valor) {
  serialPtr->println(valor);
}

*/






