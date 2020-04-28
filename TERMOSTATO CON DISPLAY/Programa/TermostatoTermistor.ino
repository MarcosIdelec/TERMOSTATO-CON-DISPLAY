#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "DFRkeypad.h"
#define ThermistorPIN 1  // Analog Pin 1
#define NUM_KEYS 5

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
 *
 * Esquema del sensor de temperatura:
 *   [Ground] -- [10k-pad-resistor] -- | -- [10k thermistor] --[Vcc (5v)]
 *                                     |
 *                                Analog Pin 1
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
boolean luzEncendida=true;
boolean cursorActivo=false;
boolean enMenu=false;
unsigned long time;
unsigned long tiempoPID;
byte numeroLecturas=0;
int x=0;
int signo=0;
char temp[10];
int lecturas[100];
byte numeroLectura=0;
//int lectura=0;
int maximo, minimo, diferencia, t1, t2, t3;
byte consigna=25;
byte tiempo=1;
byte kp=1;
byte kd=1;
byte ki=1;
byte intensidad=10;


void setup()
{
  cargarConfig();
  pinMode(10, OUTPUT);
  analogWrite(10,intensidad*25);
  Serial.begin(9600); 
  lcd.clear(); 
  lcd.begin(16, 2);
  lcd.setCursor(0,0); 
  lcd.print("www.ajpdsoft.com");
  lcd.setCursor(0,1); 
  lcd.print("C.Ventanas v1.0 ");
  delay(2000);
  //  lcd.setCursor(0,0); 
  //  lcd.print("Muevase con las ");
  //  lcd.setCursor(0,1); 
  //  lcd.print("teclas direccion");
  //  delay(4000);
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Temperatura:    ");
  lcd.setCursor(0,1);
  int lectura=getTemp(1);
  sprintf(temp, "%3d%c%1d%cC", lectura/100, '.', lectura/10%10,223);
  lcd.print(temp);
  time = millis();
  tiempoPID = millis();
}

void loop()
{
  int lectura = getTemp(ThermistorPIN);
  lecturas[numeroLectura++] = lectura;
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
    sprintf(temp, "%3d%c%1d%cC", lectura/100, '.', lectura/10%10,223);
    lcd.print(temp);
    //    Serial.print("Temperatura[");
    //    Serial.print(numeroLectura);
    //    Serial.print("]: "); 
    //    Serial.println(temp); 
  }
  if (millis()-time > 5000) {  // Si han pasado mas de 5 segundos apagamos el cursor
    lcd.noBlink();
    cursorActivo=false;
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
          lcd.print("0 seg.");
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
  }
  if ((numeroLectura > 99) && (numeroLecturas < 2)) {
    long suma = 0;
    maximo = -10000;
    minimo = 10000;
    for (int i=0; i < 100; i++){
      suma = suma + lecturas[i];
      if (lecturas[i] > maximo) {
        maximo = lecturas[i];
      }
      if (lecturas[i] < minimo) {
        minimo = lecturas[i];
      }

      //      Serial.print("Temperatura[");
      //      Serial.print(i);
      //      Serial.print("]: "); 
      //      Serial.println(lecturas[i]); 
    }
    diferencia = maximo - minimo;
    suma = suma / 100;
    // Si la diferencia es superior a un grado 
    //es que ha habido un error en la lectura
    if (diferencia > 100) {  
      Serial.println("Lectura no valida");
      // TODO: Descartar lectura y repetir la medida
    } 
    else {
      numeroLecturas++;
      t3=t2;
      t2=t1;
      t1=suma;
    }
    Serial.print("Suma: "); 
    Serial.println(suma); 
/*    Serial.print("Media: "); 
    Serial.print(suma/100); 
    Serial.print(","); 
    Serial.println(suma/10%10); 
    Serial.print("Maximo: "); 
    Serial.print(maximo/100); 
    Serial.print(","); 
    Serial.print(maximo/10%10); 
    Serial.print(" - Minimo: "); 
    Serial.print(minimo/100); 
    Serial.print(","); 
    Serial.print(minimo/10%10); 
    Serial.print(" - Diferencia: "); 
    Serial.print(diferencia/100); 
    Serial.print(","); 
    Serial.println(diferencia/10%10); */
  }
  if (numeroLectura > 99) {
    // Cuando se termina de tomar las 100 
    //lecturas empezamos de nuevo por la primera
    numeroLectura = 0;
  }
  PID();
  delay(10);
  // Si se desborda millis() empieza otra 
  //vez por cero, ocurre cada 50 dias
  if (millis() < time){   
    time = millis();
  }
}

void PID(){
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
    Serial.print("Distancia a la consigna: ");
    Serial.print(t1-consigna*100);
    Serial.print(" - Velocidad: ");
    Serial.println(t1-t2);
    tiempoPID = millis();
  }

}

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
int getTemp(byte input)
{
  int  celsius = Thermistor(analogRead(input))*100; 
  return celsius;
  temperaturaFicticia += (random(7)-3-ptf);
  if (temperaturaFicticia > 2800) ptf = 1;
  if (temperaturaFicticia < 2000) ptf = 0;
  return temperaturaFicticia;
}

float pad = 10000;  // balance/pad resistor value, set this to
                   // the measured resistance of your pad resistor
// float thermr = 10000; // thermistor nominal resistance

float Thermistor(int RawADC) {
  long Resistance;  
  float Temp;  // Dual-Purpose variable to save space.

  Resistance=((1024 * pad / RawADC) - pad); 
  // Saving the Log(resistance) so not to calculate  it 4 times later
  Temp = log(Resistance); 
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + 
         (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convert Kelvin to Celsius                      

  return Temp;    // Devolver temperatura
}

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
 
