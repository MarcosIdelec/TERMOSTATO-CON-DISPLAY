//**********************************************************************************************************************************************************
//**********************************************************************************************************************************************************
// Fecha 30/04/2020   11:57
//**********************************************************************************************************************************************************
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "DFRkeypad.h"
#include <Adafruit_MAX31865.h>
#include <PID_v1.h>

#define Salida  // Analog Pin 3
#define NUM_KEYS 5
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(2, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
//#define PIN_INPUT 0
//#define RELAY_PIN 6
/*
  Circuito del dispaly LCD:
 * LCD RS pin to digital pin 8
 * LCD Enable pin to digital pin 9
 * LCD D4 pin to digital pin 4
 * LCD D5 pin to digital pin 5
 * LCD D6 pin to digital pin 6
 * LCD D7 pin to digital pin 7
 * LCD BL pin to digital pin 10
 * KEY pin to analogl pin 0
 */
LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);
const int numeroDeMenus=6;
char tituloMenu[numeroDeMenus][16] = {
  "Fijar Temp.: ",
  "Fijar Tiempo:",               
  "Kp:          ",
  "Kd:          ",
  "Ki:          ",
  "Intensidad:  " };
int adc_key_val[5] ={
  50, 200, 400, 600, 800 };
int adc_key_in;
int key=-1;
int oldkey=-1;
int x=0;
int signo=0;
int maximo, minimo, diferencia, t1, t2, t3;
int lecturas[100];
//int WindowSize = 5000;

boolean luzEncendida=true;
boolean cursorActivo=false;
boolean enMenu=false;

unsigned long time;
unsigned long tiempoPID;                    //Tiempo asignado al PID

uint16_t rtd=0; 
uint16_t lectura=0;

byte numeroLectura=0;
byte numeroLecturas=0;
byte consigna=25;
byte tiempo=1;
byte kp=1;
byte kd=1;
byte ki=1;
byte intensidad=80;

//const int ledPin =  3;                                    //salida proporcional.
const int RELAY_PIN =  3;                                    //salida proporcional.
 
long suma=0;
long media=0;
char temp[10];
float Temp=0;
 
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

//**********************************************************************************************************************************************************
//**********************************************************************************************************************************************************
// SETUP
//**********************************************************************************************************************************************************
void setup()
{
 
  //----------------------------------------------------------------------------- setup Sonda
  Serial.begin(115200);
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");
  //thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary Original programa pero compila con ....
  thermo.begin(MAX31865_4WIRE);    // set to 2WIRE or 4WIRE as necessary 
  //----------------------------------------------------------------------------- setup teclado
 pinMode(RELAY_PIN, OUTPUT);
 digitalWrite(RELAY_PIN, HIGH);

boolean  cargarConfig();
  pinMode(10, OUTPUT);
 
  analogWrite(10,intensidad*25);

  lcd.clear(); 
  lcd.begin(16, 2);
  lcd.setCursor(0,0); 
  lcd.print("www.idelec.com");
  lcd.setCursor(0,1); 
  lcd.print("M.Fernandez v1.0 ");
  delay(1000);
  lcd.setCursor(0,0); 
  lcd.print("Muevase con las ");
  lcd.setCursor(0,1); 
  lcd.print("teclas direccion");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Temperatura:    ");
  lcd.setCursor(0,1);

  lcd.print("Inicializando....");
  time = millis();                       // inicializaos tiempo
  tiempoPID = millis();                  // inicializaos tiempo
  //---------------------------------------------------------------------------------------
  windowStartTime = millis();

  //initialize the variables we're linked to
  // Setpoint = 100;
     Setpoint = -7;  


  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}
//*****************************************************************************************************************
//************************************************************************************************************************************************** LOOP
//*****************************************************************************************************************

void loop()
{
  // Input = analogRead(PIN_INPUT);
  // myPID.Compute();
  
  uint16_t rtd = thermo.readRTD();
 // Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
/*
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperatura = "); Serial.println(thermo.temperature(RNOMINAL, RREF));
  */
  float lectura = thermo.temperature(RNOMINAL, RREF);

  lecturas[numeroLectura++] =  int(lectura *100);
  /*  
  Serial.print(" LECTURAS TEMP ...... : "); Serial.println  (float (lectura));
  Serial.print(" Numero de LECTURA...... : "); + Serial.println (numeroLectura);   
  Serial.print(" Numero de LECTURaSSSSS..... : "); + Serial.println (lecturas[numeroLectura]);
  Serial.print(" Numero de LECTU*100..... : "); + Serial.println (int (lectura *100));
*/
  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }
  //Serial.println();
  //delay(10);
        //--------------------------------------------------------------------------------------------------------------------------


  if (millis()-time > 20000) {  // Si han pasado mas de 20 segundos apagamos la luz
    digitalWrite(10, LOW);
    luzEncendida=false;
  }
  
  if (millis()-time > 7000) {  // Si han pasado mas de 7 segundos salimos del menu
    if (enMenu) guardarConfig();
    enMenu = false;
    x=0;
    time = millis();
    //  }
    //  if (!enMenu) {
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print("Temperatura:    ");
    lcd.setCursor(0,1);
 //   sprintf(temp, "%3d%c%1d%cC", lectura/100, '.', lectura/10%10,223);
    lcd.print(float (lectura));
    
  /*   Serial.print("Temperatura[");
       Serial.print(numeroLectura);
       Serial.print("]: "); 
       Serial.println(temp); 
       */
  }

  
  if (millis()-time > 5000) {  // Si han pasado mas de 5 segundos apagamos el cursor
    lcd.noBlink();
    cursorActivo=false;
  } 



  adc_key_in = analogRead(0);    // Leemos el valor de la pulsacion
  key = get_key(adc_key_in);    // Obtenemos el boton pulsado

  if (key != oldkey)   // if keypress is detected
  {
    delay(40);  // Espera para evitar los rebotes de las pulsaciones
    adc_key_in = analogRead(0);    // Leemos el valor de la pulsacion
    key = get_key(adc_key_in);    // Obtenemos el boton pulsado
    if (key != oldkey)    
    {
      time = millis();  // TODO: falta la comprobacion de si se ha desbordado el tiempo
      if (!luzEncendida) {  // Al pulsar cualquier tecla encendemos la pantalla
        analogWrite(10,intensidad*25);
        luzEncendida=true;
      } 
      else {  // si la pantalla esta encendida seguimos funcionando normalmente
        oldkey = key;
        char accion = 0;
        if (key >=0){  // Si se ha pulsado cualquier tecla
          lcd.blink();  // Mostramos el cursor parpadeando
          cursorActivo=true;
        }
        if ((key == 0) && (enMenu)){  // Se ha pulsado la tecla derecha
          x++;
          if (x>numeroDeMenus-1) x=numeroDeMenus-1;
        }
        if ((key == 1) && (enMenu)) {  // Se ha pulsado la tecla arriba
          accion++;
        }
        if ((key == 2) && (enMenu)) {  // Se ha pulsado la tecla abajo
          accion = accion-1;
        }
        if ((key == 3) && (enMenu)) {  // Se ha pulsado la tecla izquierda
          x--;
          if (x<0) x = 0;
        }
        if (key == 4){  // Se ha pulsado la tecla de seleccion
        }
        enMenu = true;
        lcd.clear();
        lcd.setCursor(0,0); 
        lcd.print(tituloMenu[x]);
        lcd.setCursor(0,1); 
        switch (x) {
        case 0: // Estamos en fijar temperatura
          consigna += accion;
          lcd.print(consigna);
          lcd.print((char)223);
          lcd.print("C");
          break;
        case 1:  // Estamos en fijar tiempo
          tiempo += accion;
          lcd.print(tiempo);
          lcd.print(" seg.");
          break;
        case 2:  // Estamos en Kp.
          kp += accion;
          lcd.print(kp);
          break;
        case 3:  // Estamos en Kd.
          kd += accion;
          lcd.print(kd);
          break;
        case 4:  // Estamos en Ki.
          ki += accion;
          lcd.print(ki);
          break;
        case 5:  // Estamos en Ki.
          intensidad += accion;
          if (intensidad > 254) intensidad = 0;
          if (intensidad > 10) intensidad = 10;
          lcd.print(intensidad);
          lcd.print("0%");
          analogWrite(10,intensidad*25);
          break;
        } 
      }
    }
  }        // Teminación de las acciones de teclado.

Setpoint = consigna;
Setpoint = (consigna -(consigna*2));
WindowSize = tiempo;
Kp = kp;
Kd = kd;
Ki = ki;
Serial.print("consigna =");Serial.println(Setpoint); 
Serial.print("tiempo =");Serial.println(WindowSize);
Serial.print("kp =");Serial.println(Kp);
Serial.print("kd =");Serial.println(Kd);
Serial.print("ki =");Serial.println(Ki);   


 //   ***********************************************************************************  Iniciamos el muestreo de las 100 muestras.
  //if ((numeroLectura > 99) && (numeroLecturas < 2)) 
  if (numeroLectura > 9) 
    {
          //Si estamos en la muestra 1  ponemos la suma maximo y minimo a sus valoras maximos de escala  
    long suma = 0;
    maximo = -10000;
    minimo = 10000;
          //Entre la muestra 1 y la 100 se acumula un sumando de las muestras 
    for (int i=0; i < 10; i++){
      suma = suma + lecturas[i];
          //Serial.println(suma); 
                  if (lecturas[i] > maximo) {
                      maximo = lecturas[i];         //Si hay un nuevo maximo se guarda como nuevo maximo
                      }
                   if (lecturas[i] < minimo) {
                       minimo = lecturas[i];         //Si hay un nuevo minimo se guarda como nuevo minimo
                      }

       /*   Serial.print("Temperatura[");
          Serial.print(i);
          Serial.print("]: "); 
          Serial.println(lecturas[i]); 
          */
      // lineas comentadas que descomento para probar
        
        }

        
    diferencia = maximo - minimo;    // De las 100 medidas obtenemos una diferencia maxima   
  
    //
            if (diferencia > 100) {               // Si la diferencia es superior a un grado consideramos que ha habido un error en la lectura
                Serial.println("Lectura no valida");
                     //  Descartar lectura y repetir la medida
              } 
             else {
                      numeroLecturas++;     //Pero si la diferencia es acepable damos por buena una medida con esa media.
                       t3=t2;
                        t2=t1;
                        t1=suma;
             }
    //Por lo tanto despues de 100 muestra tenemos los siguientes datos.
    Serial.println("**********************************************************************************************************");
    Temp = float(suma/1000.0);
    Serial.print("Temp: "); 
    Serial.println(Temp);
    //Serial.println(float(suma/10000.0));
    Serial.print("Suma: "); 
    Serial.println(suma); 
    Serial.print("Media: "); 
    Serial.println(float(suma/10.0)); 
    //Serial.print(","); 
    //Serial.println(suma/10%10); 
    Serial.print("Maximo: "); 
    Serial.println(maximo); 
    //Serial.print(","); 
    //Serial.print(maximo/10%10); 
    Serial.print("Minimo: "); 
    Serial.println(minimo); 
    //Serial.print(","); 
    //Serial.print(minimo/10%10); 
    Serial.print("Diferencia: "); 
    Serial.println(diferencia); 
    //Serial.print(","); 
    //Serial.println(diferencia/10%10);
    Serial.println("**********************************************************************************************************");
  Input = lectura;
  myPID.Compute(); 
   // PID();  //rutina
   delay(10); 

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime) digitalWrite(RELAY_PIN, HIGH);
  else digitalWrite(RELAY_PIN, LOW);

  
  // funcioncontrol(float (Temp));   // Salto al control
  
  }
//***************************************************** Terminamos el muestroeo de las 100 muestras
  
  if (numeroLectura > 9) {
    // Cuando se termina de tomar las 100 
    //lecturas empezamos de nuevo por la primera
    Serial.println("BUCLE DE 10 LECTURAS  TERMINADO : ");
    numeroLectura = 0;
   
  }


 // delay(10);      
  // Si se desborda millis() empieza otra 
  //vez por cero, ocurre cada 50 dias
  if (millis() < time){   
    time = millis();
  }
}
//*****************************************************************************************************************
//********************************************************************************************     TERMINA       LOOP
//*****************************************************************************************************************

void PID(){
  Serial.println("EJECUTANDO PID: ");
  // Si se desborda millis() empieza otra 
  // vez por cero, ocurre cada 50 dias
          if (millis() < tiempoPID){   
              tiempoPID = millis();
          }
             // Si no ha pasado todavía el timepo de ciclo del PID
          if (millis() < tiempoPID + (tiempo*10*1000)){  
             // entonces mantenemos la fuerza y esperamos mas tiempo
   
            // s = sActualPID;  
          } 
          else if (numeroLecturas >= 2){
            
          numeroLecturas--;
          Serial.print("Numero lecturas: "); Serial.println(numeroLecturas); 
          Serial.print("Distancia a la consigna: ");
          Serial.print(t1-consigna*100);
          Serial.print(" - Velocidad: ");
          Serial.println(t1-t2);
          tiempoPID = millis();
          }

}
//----------------------------------------------------------------------------------------------------  

// Convertimos el valor leido en analogico 
// en un numero de boton pulsado
int get_key(unsigned int input)
{
  int k;

  for (k = 0; k < NUM_KEYS; k++)
  {
    if (input < adc_key_val[k])
    {
      return k;
    }
  }

  if (k >= NUM_KEYS)k = -1;  // Error en la lectura.
  return k;
}




int temperaturaFicticia = 2450;
int ptf= 0;

//**********************************************************************************************************************************************************
//**********************************************************************************************************************************************************
//**********************************************************************************************************************************************************

boolean cargarConfig(){
  if ((EEPROM.read(0) == 27) && (EEPROM.read(1) == 28) && 
     (EEPROM.read(2) == 13) && (EEPROM.read(3) == 18)) {
    // Comprobamos que la eeprom tenga una 
    // configuracion valida con numeros concretos
    // solo cargamos el valor de la configuracion si los valores coinciden
    if (EEPROM.read(4) == EEPROM.read(5)) consigna = EEPROM.read(4);  
    if (EEPROM.read(6) == EEPROM.read(7)) tiempo = EEPROM.read(6);
    if (EEPROM.read(8) == EEPROM.read(9)) kp = EEPROM.read(8);
    if (EEPROM.read(10) == EEPROM.read(11)) kd = EEPROM.read(10);
    if (EEPROM.read(12) == EEPROM.read(13)) ki = EEPROM.read(12);
    if (EEPROM.read(14) == EEPROM.read(15)) intensidad = EEPROM.read(14);
    return true;
  }
  return false;
}

void guardarConfig(){
  EEPROM.write(0,27);
  EEPROM.write(1,28);
  EEPROM.write(2,13);
  EEPROM.write(3,18);
  // Ponemos nmeros concretos en el comienzo 
  // de la EEPROM para confirmar que tiene valores correctos.
  EEPROM.write(4,consigna);
  EEPROM.write(5,consigna);  // almacenamos los valores 2 veces
  EEPROM.write(6,tiempo);
  EEPROM.write(7,tiempo);  // almacenamos los valores 2 veces
  EEPROM.write(8,kp);
  EEPROM.write(9,kp);  // almacenamos los valores 2 veces
  EEPROM.write(10,kd);
  EEPROM.write(11,kd);  // almacenamos los valores 2 veces
  EEPROM.write(12,ki);
  EEPROM.write(13,ki);  // almacenamos los valores 2 veces
  EEPROM.write(14,intensidad);
  EEPROM.write(15,intensidad);  // almacenamos los valores 2 veces
}

//----------------------------------------------------------------------------------------------------
float funcioncontrol(float Temp) {
 
 int consigna = -7.0;
 Serial.println("ESTOY EN CONTROL: ");
 Serial.print("ME HAS PASADO UN ..... "); Serial.println (Temp);
// delay(1000); 

   Serial.print(" Temp..="); Serial.println(Temp);
   Serial.print("Consigna:"); Serial.println(consigna);
if (Temp >= 0){
   
    if (Temp >= consigna) {  
    // entonces mantenemos la fuerza y esperamos mas tiempo
   Serial.println("Sigo enfriando:");
   digitalWrite (3, HIGH); 
      //delay(1000); 
        } 
        else  {
          Serial.println("Dejo de enfriar: ");
          digitalWrite (3, LOW); 
           // delay(1000); 
              } 
            }  
else {                // Si la temp es negativa
    if (Temp <= consigna) {  
    // entonces mantenemos la fuerza y esperamos mas tiempo
   Serial.println("Dejo de enfriar:");
   digitalWrite (3, LOW); 
      //delay(1000); 
        } 
        else  {
          Serial.println("Sigo enfriando : ");
          digitalWrite (3, HIGH); 
           // delay(1000); 
              } 
      }           
}


/********************************************************
 * PID RelayOutput Example
 * Same as basic example, except that this time, the output
 * is going to a digital pin which (we presume) is controlling
 * a relay.  the pid is designed to Output an analog value,
 * but the relay can only be On/Off.
 *
 *   to connect them together we use "time proportioning
 * control"  it's essentially a really slow version of PWM.
 * first we decide on a window size (5000mS say.) we then
 * set the pid to adjust its output between 0 and that window
 * size.  lastly, we add some logic that translates the PID
 * output into "Relay On Time" with the remainder of the
 * window being "Relay Off Time"
 ********************************************************/
