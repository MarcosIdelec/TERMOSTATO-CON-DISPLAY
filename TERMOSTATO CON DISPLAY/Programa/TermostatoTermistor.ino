//**********************************************************************************************************************************************************
//**********************************************************************************************************************************************************
// Fecha 05/05/2020   11:57
// Autor Marcos Fernández
// IDE arduino 1.8.12
// Placa Arduino Mini o ProMINI
//**********************************************************************************************************************************************************
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "DFRkeypad.h"
#include <Adafruit_MAX31865.h>
#include <PID_v1.h>
#define Salida  // Analog Pin 3
#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0
#define REVERSE 1
int controllerDirection=REVERSE;
#define NUM_KEYS 5
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(2, 11, 12, 13);
//use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);
const int numeroDeMenus=10;
char tituloMenu[numeroDeMenus][16] = {
  "  EJECUTAR   ",
  "Temp.Cons.1: ",
  "Temp.Cons.2: ",
  "PENDIENTE .: ",
  "Fijar Tiempo:",               
  "Kp:          ",
  "Kd:          ",
  "Ki:          ",
  "Intensidad:  ",
  "Grabar datos ",};
int adc_key_val[5] = { 50, 200, 400, 600, 800 };
// int adc_key_val[5] = { 50, 150, 400, 800, 980 }; //Parametros para tarjeta roja promini
int IntMensg=1;
int adc_key_in;
int key=-1;
int oldkey=-1;
int x=0;
int signo=0;
int maximo, minimo, diferencia, t1, t2, t3;
int lecturas[10];
int WindowSize = 0;
int EjeProg= 0;
int alcanzado;
int estabilizado;
boolean luzEncendida=true;
boolean cursorActivo=false;
boolean enMenu=false;
boolean Reinicio = false;
unsigned long time;
unsigned long tiempoPID;   //Tiempo asignado al PID
uint16_t rtd=0; 
uint16_t lectura=0;
byte numeroLectura=0;
byte numeroLecturas=0;
byte tiempo = 1;
unsigned long TiempoAcumulado1=0;
unsigned long TiempoAcumulado2=0;
unsigned long IncTiempoEnConsigna1=0;
unsigned long IncTiempoEnConsigna2=0;
unsigned long TiempoInterRampa=0;
unsigned long TiempoEscalon=0;
byte intensidad=80;
int confirmar = 0;                                   //salida proporcional.
const int RELAY_PIN =  3;                                    //salida proporcional.
long suma=0;
long media=0;
char temp[10];
float TempMedida=0;
float TempMedAnterior=0;
float TempParcial=0;
//La libreria PID utiliza Setpoint, Input, Output;
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=0, Ki=-0, Kd=0;
int SampleTime = 1000;//Seteamoseltiempodemuestreoen1segundo.
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);   //Reverse porque utilizamos medida negativa
unsigned long windowStartTime;
double Setpoint1=0;
double Setpoint2=0;
int GradMint=0;


struct MyObject {
  double field1;  // Primer punto consigna
  double field2;  // Segundo punto consigna
  double field3;  // Grados/ minuto
  int field4;     //tiempo  Ventana medicion;
  double field5;
  double field6;
  double field7;
  byte  field8;        
};


//************************************************************************************************************
//*************************************************************************************************************
// SETUP
//*************************************************************************************************************
void setup()
{
 //----------------------------------------------------------------------------- setup Sonda
  Serial.begin(115200);
  thermo.begin(MAX31865_4WIRE);    // set to 2WIRE or 4WIRE as necessary 
  //----------------------------------------------------------------------------- setup teclado
 pinMode(RELAY_PIN, OUTPUT);
 digitalWrite(RELAY_PIN, LOW);

 // cargarConfig (double (Setpoint1),double (Setpoint2),int (GradMint),int (WindowSize),double (Kp),double (Kd),double (Ki),byte (intensidad));
  cargarConfig ();
 
  Serial.println("Read custom object from EEPROM: ");
  Serial.println(double (Setpoint1));
  Serial.println(double (Setpoint2));
  Serial.println(int (GradMint));
  Serial.println(int (WindowSize));
  Serial.println(double (Kp));
  Serial.println(double (Kd));
  Serial.println(double (Ki));
  Serial.println(byte (intensidad));
   

  Serial.println(" CARGADA CONFIGURACIÓN"); 
  pinMode(10, OUTPUT);
  analogWrite(10,intensidad*25);
  lcd.clear(); 
  lcd.begin(16, 2);
  lcd.setCursor(0,0); 
  lcd.print("www.idelec.com");
  lcd.setCursor(0,1); 
  lcd.print("M.Fernandez v1.0 ");
  //delay(1000);
  lcd.setCursor(0,0); 
  lcd.print("Muevase con las ");
  lcd.setCursor(0,1); 
  lcd.print("teclas direccion");
  //delay(3000);

  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Inicio OK Selec.");
  lcd.setCursor(0,1);
  lcd.print("izq arb abj der ");
  delay(3000);

  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("<--Int.Inicio   ");
  lcd.setCursor(0,1);
  lcd.print("Menu Datos -->");
delay(3000);
  
  time = millis();                       // inicializaos tiempo
  tiempoPID = millis();                  // inicializaos tiempo
  windowStartTime = millis();
  WindowSize = 1000;
  Setpoint=0;
 // tiempo=WindowSize ; 
  controllerDirection = REVERSE ;
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);
  // SetTunings(Kp, Ki, Kd);
  EjeProg= false;
}
//*****************************************************************************************************************
//**************************************************************************************************** LOOP
//*****************************************************************************************************************

void loop()
{
  //------------------------------------------------------------------------- Atiendo el sensor reviso rotura o alarma
   uint16_t rtd = thermo.readRTD();
 // Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
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
//---------------------------------------------------------------------------  hasta aqui el tema sonda  

//---------------------------------------------------------------------------------------------Atendemos el teclado
confirmar = 0 ;      //ponrmos el flag de confirmar de las opciones menu 9 y 10 a 0 en cada entrada a la rutina teclado


if (millis()-time > 20000) {  // Si han pasado mas de 20 segundos apagamos la luz
  
   analogWrite(10,40);  // Ponemos el display en penumbra
   
    luzEncendida=false;
  }

//----------------------------------------------------------------------------------------------------------------------
  
    if (millis()-time > 7000) {  // Si han pasado mas de 7 segundos salimos del menu
    enMenu = false;
    x=0;
 if ((EjeProg==0)&& (Reinicio==0 )) {
  Reinicio=1;
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("<--Int.Inicio   ");
  lcd.setCursor(0,1);
  lcd.print("Menu Datos -->"); 
  }

 
  }
  if (millis()-time > 5000) {  // Si han pasado mas de 5 segundos apagamos el cursor
    lcd.noBlink();
    cursorActivo=false;
  } 
if (millis()-time > 20000) {  // Si han pasado mas de 20 segundos apagamos la luz
  analogWrite(10,intensidad*20);
  luzEncendida=false;
  }
  adc_key_in = analogRead(0);    // Leemos el valor de la pulsacion
  key = get_key(adc_key_in);    // Obtenemos el boton pulsado   

  if (key != oldkey)   // if keypress is detected
  {
    delay(50);  // Espera para evitar los rebotes de las pulsaciones
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
          if ((key == 4) && (enMenu)) {    // Se ha pulsado la tecla de seleccion
           confirmar = 1;                          
        }
        enMenu = true;
        lcd.clear();
        lcd.setCursor(0,0); 
        lcd.print(tituloMenu[x]);
        lcd.setCursor(0,1); 
        switch (x) {
        case 1: // Estamos en fijar Temp.Cons.1
          Setpoint1 += accion;
          //Setpoint1 = (0-Setpoint1);
          lcd.print(Setpoint1);
          lcd.print((char)223);
          lcd.print("C");
          break;
         case 2:  // Estamos en fijar Temp.Cons.2
          Setpoint2 += accion;
          if (Setpoint2 < -40) intensidad = -40;
          if (Setpoint2 > 0) intensidad = 0;
          //Setpoint2 = (0-Setpoint2);
          lcd.print(Setpoint2);
          lcd.print((char)223);
          lcd.print("C");
          break;
         case 3:  // Estamos Grados minuto
          
          GradMint += accion;
          if (GradMint <0 ) GradMint = 0;
          if (GradMint >9 ) GradMint = 9;
          lcd.print("0.");
          lcd.print(GradMint);
          lcd.print((char)223);
          lcd.print(" C Grd/Mint");
          break;
        case 4:  // Estamos en fijar Tiempo Rampa
         tiempo += accion;
          WindowSize = (tiempo*1000);
          lcd.print(WindowSize);
          lcd.print(" mseg.");
          break;
        case 5:  // Estamos en Kp.
          Kp += accion;
          
          lcd.print(Kp);
          break;
        case 6:  // Estamos en Kd.
          Kd += accion;
           
          lcd.print(Kd);
          break;
        case 7:  // Estamos en Ki.
          Ki += accion;
           
         lcd.print(Ki);
          break;
        case 8:  // Estamos en Intensidad.
          intensidad += accion;
          if (intensidad > 254) intensidad = 0;
          if (intensidad > 10) intensidad = 10;
          lcd.print(intensidad);
          lcd.print("0%");
          analogWrite(10,intensidad*25);
           break;
         case 9:  // Estamos confirmar.
          confirmar += accion;
          if (confirmar > 1) confirmar = 1;
          if (confirmar < 0 ) confirmar = 0;
          
          lcd.print(" SELECT= Grabar ");
                 if (confirmar == 1) { 
                               lcd.clear();
                               lcd.setCursor(0,0); 
                               lcd.print("   Grabando... ");
                               delay (1000);
                               Serial.print("Ejecutando grabación =");Serial.println(confirmar);
                     guardarConfig();    // llamo a guardar config
                               Serial.print("Grqabación efectuada =");Serial.println(confirmar);
  
                          }      
          break;
                 case 0:  // Estamos ejecutar.
          confirmar += accion;
          if (confirmar > 1) confirmar = 1;
          if (confirmar < 0 ) confirmar = 0;
          
          lcd.print(" <--  PROGRAMA  ");
                 if (confirmar == 1 && estabilizado == 0) { 
                               lcd.clear();
                               lcd.setCursor(0,0); 
                               lcd.print(" INICIO EJECUCION ");
                               delay (1000);
                               Serial.print("Ejecutando programa de enfriamiento =");Serial.println(confirmar);
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print("Ejecutando Prog.:");
    lcd.setCursor(0,1);
    lcd.print("Emp..Enfriamiento");
                     EjeProg = 1;     // Ejecutando ciclo enfriamiento                     
                     alcanzado = 0; 
                     Setpoint=Setpoint1;//
    }
                if (confirmar == 1 && estabilizado == 1)  {
                      
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print("Ejecutando Prog.:");
    lcd.setCursor(0,1);
    lcd.print("Inc.Descenso Rampa");
            IncTiempoEnConsigna2 = millis();
            TiempoInterRampa=0;
            TempMedAnterior=TempMedida ;
            Setpoint=Setpoint2;
            delay (1000);
    EjeProg = 2;
                
                  
                  }
                            
          break;  
        } 
      }
    }
  
 
  }        // ---------------------------------------------------------------------------- Tecla pulsada .
  
// Serial.print("Time=");Serial.println(time);  
// Serial.print("EjeProg..=");Serial.println(boolean(EjeProg)); 

  
//   ************************************************************ Si ya hay 10 lecturas sacamos la media
  if (numeroLectura > 9) 
    { 
    long suma = 0;
    maximo = -10000;
    minimo = 10000;
          //Entre la muestra 1 y la 10 se acumula un sumando de las muestras 
    for (int i=0; i < 10; i++){
      suma = suma + lecturas[i];
          //Serial.println(suma); 
                  if (lecturas[i] > maximo) {
                      maximo = lecturas[i];         //Si hay un nuevo maximo se guarda como nuevo maximo
                      }
                   if (lecturas[i] < minimo) {
                       minimo = lecturas[i];         //Si hay un nuevo minimo se guarda como nuevo minimo
                      }

      // lineas comentadas que descomento para probar
        
        }
       diferencia = maximo - minimo;    // De las 10 medidas obtenemos una diferencia maxima   
    
            if (diferencia > 100) {               // Si la diferencia es superior a un grado consideramos que ha habido un error en la lectura
                //Serial.println("Lectura no valida");
                     //  Descartar lectura y repetir la medida
              } 
             else {
                      numeroLecturas++;     //Pero si la diferencia es acepable damos por buena una medida con esa media.

             }
  
TempMedida=float(suma/1000.0);
//--------- Aqui ya tenemos la medida efectuada en el buclee loop una media de 10 lecturas simples y el resto datos para tomar decisiones
/*
Serial.println("*******************************************************************");
Serial.print("Kp ");Serial.println(Kp);
Serial.print("Ki ");Serial.println(Ki);
Serial.print("Kd ");Serial.println(Kd);
Serial.print("VENTANA TIEMPO ");Serial.println(WindowSize);
Serial.print("TIEMPO INICIO ");Serial.println(windowStartTime); 
Serial.print("PUNTO Setpoint ");Serial.println(Setpoint);
Serial.print("OUT PID ");Serial.println (Output); 
Serial.print("TIEMPO ");Serial.println (millis());
Serial.print("TEMPERATURA REAL "); Serial.println(Temp);
Serial.print("TIEMPO ");Serial.println (millis());  
Serial.print("PUNTO Setpoint ");Serial.println(Setpoint);
Serial.print("TEMPERATURA REAL "); Serial.println(TempMedida);
Serial.print("OUT PID ");Serial.println (Output);  
Serial.println("*******************************************************************");
*/

//--------------------------------------------------------------------------Si estoy ejecutando primera parte programa EjeProg == 1
//**************************************************************************************************************************************
//**************************************************************************************************************************************

if (EjeProg == 1) {
    Serial.println("ESTOY EN FASE    1    ="); 
    Setpoint=Setpoint1;
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print("<Enfriando><Tep>");
    lcd.setCursor(0,1);
    lcd.print(float (Setpoint1));
    lcd.setCursor(10,1);
    lcd.print(float (TempMedida));
   
             funcioncontrol(float (TempMedida), double (Setpoint));

      if (float (TempMedida) < double (Setpoint1)) {
         Serial.print("Alcanzada temperatura consigna 1 =");Serial.println(alcanzado); 
                      if  (alcanzado == 0) {   // Si es la primera vez y alcanzado es = 0 entonces
                            IncTiempoEnConsigna1  = millis();       //Inicializo el Inicio Tiempo en consigna1
 
                        Serial.print("IncTiempoEnConsigna1  =");Serial.println(IncTiempoEnConsigna1);
                            alcanzado = 1;    
                                  }
    if (alcanzado == 1) {
          lcd.clear();
          lcd.setCursor(0,0); 
         TiempoAcumulado1 = millis() - (IncTiempoEnConsigna1 );   // El tiempo en estado 1 se va midiendo
    Serial.print("IncTiempoEnConsigna1  =");Serial.println(IncTiempoEnConsigna1); 
    Serial.print("TiempoAcumulado1  =");Serial.println(TiempoAcumulado1); 
    Serial.print("millis  =");Serial.println (long(millis())); 
    lcd.print("<ALCANZADO><Tep>");
    lcd.setCursor(0,1);
    lcd.print(float (Setpoint1));
    lcd.setCursor(10,1);
    lcd.print(float (TempMedida));
          if (TiempoAcumulado1 > 30000) {         // TIEMPO PARA CONSIDERAR LA MEDIDA ESTABLE ESPERA EN FASE 1 
                           
                                  Serial.print("ALCANZADA ESTABILIDAD");
                                  estabilizado = 1;
                     if (IntMensg ==1) {
                                            lcd.clear();
                                            lcd.setCursor(0,0);    
                                            lcd.print("< ESTABLE ><Tep>");
                                            lcd.setCursor(0,1);
                                            lcd.print(float (Setpoint1));
                                            lcd.setCursor(10,1);
                                            lcd.print(float (TempMedida));
                                            IntMensg=0;
                                            // switch (9); // pendiente mejora
                            }
                          else {
                                lcd.clear();
                                lcd.setCursor(0,0);    
                                lcd.print("< ESPERO ORDEN >");
                                lcd.setCursor(0,1);
                                lcd.print("<<< PULSAR INTRO");
                                IntMensg=1;
                            }  
      
                    }

        }   /// fin bucle alcanzado
    
    
   

      
}   // termina el if  temperatura alcnzada  

}   // termina el if de en programa 1


//  SI ESAMOS EN FASE 2 DEL PROGRAMA *************************************************************
//------------------------------------------------------------------------------------------------

if (EjeProg == 2) {
  //Serial.println("ESTOY EN FASE    2    =");
  //Serial.print(" GradMint  =");Serial.println(GradMint);
 
  Setpoint=Setpoint2;
             /*
                Serial.print("IncTiempoEnConsigna2  =");Serial.println(IncTiempoEnConsigna2);            
                Serial.print("TiempoInterRampa  =");Serial.println(TiempoInterRampa); 
                Serial.print("TiempoAcumulado2  =");Serial.println(TiempoAcumulado2); 
                */
    // funcioncontrol(float (TempMedida), double (Setpoint));

    
    TiempoAcumulado2 = millis() - (IncTiempoEnConsigna2 );  // SE VA INCREMENTANDO SIEMPRE QUE SE ESTE EN FASE 2
 

    if ((TiempoAcumulado2) >= (TiempoInterRampa+3000)) {         // Si han pasado mas de 60 segundos 1-MINUTO 
             Serial.println("            TIEMPO ESCALON ALCANZADO     =");   
      
              TempParcial=(TempMedida-TempMedAnterior);
              TiempoInterRampa=TiempoAcumulado2;
              
              lcd.clear();
              lcd.setCursor(0,0); 
              lcd.print("<<ESCALON><Tep>");
              lcd.setCursor(0,1);
              lcd.print(float (TempParcial));
              lcd.setCursor(10,1);
              lcd.print(float (TempMedida));
              
              
                Serial.print("TempMedAnterior  =");Serial.println(TempMedAnterior);
                Serial.print("Temperatura media =");Serial.println(TempMedida);
                Serial.print(" DIFERENCIA  =");Serial.println(TempParcial);
                
   if ( TempMedida >= TempMedAnterior) {
    Serial.print("RECTIFICANDO   =");
    TempParcial = 0;
   
   } 
                    
                funcioncontro2(float (TempParcial), int (WindowSize)); 
                TiempoInterRampa=TiempoAcumulado2;
                TempMedAnterior = TempMedida;


      }                  //  Fin condición  de medio minuto


lcd.clear();
lcd.setCursor(0,0); 
lcd.print("<Enfriando><Tep>");
lcd.setCursor(0,1);
lcd.print(float (Setpoint2));
lcd.setCursor(10,1);
lcd.print(float (TempMedida));




}  //   FIN DE BUCLE  ESTADO DE PROGRAMA EN FASE 2


// solo en caso de utilizar la rutina PID 
/* 
Input = TempMedida;       //El impud del Pid es la temp medida
Kp=(0-Kp);
Ki=(0-Ki);
Kd=(0-Kd);
//    myPID.Compute(); 
  delay(10); 
  Output =(Output*100);
*/
}   //---------------- terminamos las acciones al haber obtenido 9 lecturas y tener media de temperatura volvemos al bucle principal de lectura    

if (numeroLectura > 9) {
    //Serial.println("BUCLE DE 10 LECTURAS  TERMINADO : ");
    numeroLectura = 0; 
  }
  //vez por cero, ocurre cada 50 dias
  if (millis() < time){   
    time = millis();
  }
}
//******************************************************************************************
//********************************************     TERMINA       LOOP
//******************************************************************************************



//******************************************************************************************
//*******************  subrutinas     subrutinas    subrutinas     *************************
//******************************************************************************************

//------------------------- Convertimos el valor leido en analogico   ----------------------
//------------------------------------------------------------------------------------------ 
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
//------------------------- AUTOTUNING  ----------------------------------------------
//------------------------------------------------------------------------------------
void SetTunings(double Kp,double Ki,double Kd)
{
if(Kp<0||Ki<0||Kd<0)return;
double SampleTimeInSec=((double)SampleTime)/1000;   //Tiempo de Muestra en segundos
Kp=Kp;
Ki=Ki*SampleTimeInSec;
Kd=Kd/SampleTimeInSec;
if(controllerDirection==REVERSE)
{                     //Convierte las constantes en negativas
Kp=(0-Kp);
Ki=(0-Ki);
Kd=(0-Kd);
}
/*
Serial.print("Kp ***");Serial.println(Kp);
Serial.print("Ki ***");Serial.println(Ki);
Serial.print("Kd ***");Serial.println(Kd);
*/
Serial.print("Kp ***");Serial.println(Kp);
Serial.print("Ki ***");Serial.println(Ki);
Serial.print("Kd ***");Serial.println(Kd);
}  
//------------------------- CARGA LA CONFIGURACION EN EEPROM   ----------------------
//------------------------------------------------------------------------------------
//cargarConfig(double Setpoint1, double Setpoint2, double GradMint, int WindowSize, double Kp, double Kd, double Ki, byte intensidad)
double cargarConfig()
{
  int eeAddress = 0; //EEPROM address to start reading from
  MyObject customVar; //Variable to store custom object read from EEPROM.
  EEPROM.get( eeAddress, customVar );
    Setpoint1=(customVar.field1);
    Setpoint2=(customVar.field2);
    GradMint=(customVar.field3);
    WindowSize=(customVar.field4); 
    Kp=(customVar.field5);
    Kd=(customVar.field6);
    Ki=(customVar.field7);
    intensidad=(customVar.field8);
  Serial.println("Read custom object from EEPROM: ");
  Serial.println(customVar.field1);
  Serial.println(customVar.field2);
  Serial.println(customVar.field3);
  Serial.println(customVar.field4);
  Serial.println(customVar.field5);
  Serial.println(customVar.field6);
  Serial.println(customVar.field7);
  Serial.println(customVar.field8);
  delay(2000);
 return double (Setpoint1),double (Setpoint2),int (GradMint),int (WindowSize),double (Kp),double (Kd),double (Ki),byte (intensidad);
 //return Setpoint1,Setpoint2,GradMint,WindowSize,Kp,Kd,Ki,intensidad  ;
}

//------------------------- GUARDA LA CONFIGURACION EN EEPROM   ----------------------
//------------------------------------------------------------------------------------
void guardarConfig(){
int eeAddress = 0;   //Location we want the data to be put.
//Data to store.
  MyObject customVar = {
    Setpoint1,
    Setpoint2,
    GradMint,
    WindowSize, 
    Kp,
    Kd,
    Ki,
    intensidad,  
      };
/*  double field1;      //Temperatura consigna 1
  double field2;      //Temperatura consigna 2
  double field3;      //Grados Minuto;
  byte field4;         //Grados Minuto;
  byte field5;         //tiempo  Rampa;
  char name[10];
  */  
  EEPROM.put(eeAddress, customVar);
  Serial.print("Written custom data type! \n\nView the example sketch eeprom_get to see how you can retrieve the values!");
}

//------------------------- CONTROL NO  PID    ---------------------------------------
//------------------------------------------------------------------------------------
float funcioncontrol(float TempMedida, double Setpoint) {
/*
   Serial.println("ESTOY EN CONTROL: ");
   Serial.print(" TempMedida..="); Serial.println(TempMedida);
   Serial.print(" Setpoint:"); Serial.println(Setpoint);
*/
if (TempMedida >= 0){
   if (TempMedida >= Setpoint) {  
    // entonces mantenemos la fuerza y esperamos mas tiempo
   //Serial.println("Sigo enfriando:");
   digitalWrite (3, HIGH); 
      //delay(1000); 
        } 
        else  {
          //Serial.println("Dejo de enfriar: ");
          digitalWrite (3, LOW); 
           // delay(1000); 
              } 
            }  
else {                // Si la TempMedida es negativa
    if (TempMedida <= Setpoint) {  
    // entonces mantenemos la fuerza y esperamos mas tiempo
   //Serial.println("Dejo de enfriar:");
   digitalWrite (3, LOW); 
      //delay(1000); 
        } 
        else  {
          //Serial.println("Sigo enfriando : ");
          digitalWrite (3, HIGH); 
           // delay(1000); 
              } 
      }           
}

/* ***********************************************
// AHORA MANTENEMOS EL RELE ENCENDIDO UN TIEMPO PROPORCIONAL A LA VARIABLE 
// **********************************************/
void funcioncontro2(float TempParcial, int WindowSize)  {
   Serial.println("ESTOY EN CONTROL 2: ");
   Serial.print(" TempParcial..="); Serial.println(TempParcial);
   Serial.print(" WindowSize"); Serial.println(WindowSize);
   
   
                    if ( TempParcial >= -0.04) {Serial.print("Rampa lenta   =");Serial.println( TempParcial);
                    digitalWrite(RELAY_PIN, HIGH);
                    }
                    if ( TempParcial < -0.04 && TempParcial > -0.06) {Serial.print("Rampa CORRECTA  =");Serial.println( TempParcial);
                    digitalWrite(RELAY_PIN, HIGH);
                    }
                    if ( TempParcial <= -0.06) {Serial.print("Rampa rapida  =");Serial.println( TempParcial);
                    digitalWrite(RELAY_PIN, LOW);
                    }  


// if ( TempMedida <= TempMedAnterior) {Serial.print("Rampa lenta   =");Serial.println( TempParcial);
//digitalWrite(RELAY_PIN, HIGH);
//} 


}
//---------------------------------------------------------------------------------------------
/* ***********************************************
// turn the output pin on/off based on pid output
// *********************************************
void funcioncontro2()  {
   if (millis() - windowStartTime > WindowSize)  //Si el tiempo actual menos el tiempo inicio
                                                 //Vamos el tiemp transcurrido > que la ventana predeterminada
  { //time to shift the Relay Window
    windowStartTime += WindowSize;    // Entonces reseteamos con el nuevo tiempo que es el mismo + la ventana
  }
  //Serial.print(" milliss...="); Serial.println (int(millis));
  
  if (Output < millis() - windowStartTime) digitalWrite(RELAY_PIN, HIGH);
  else digitalWrite(RELAY_PIN, LOW);

Serial.print("millis = ");Serial.println(millis());
Serial.print("time = ");Serial.println(time);
Serial.print("windowStartTime = ");Serial.println(windowStartTime);
Serial.print("WindowSize = ");Serial.println(WindowSize);
}
*/
