// Algoritm PID pentru controlul luminozitatii.
// Se utilizeaza un fotorezistor si led.
// Procesul este alcatuit dintr-un fotorezistor si led.
// De asemenea, se masoara separat si temperatura de la un senzor DHT11, la apasarea unui buton.
// Data cand s-a creat fisierul: < 26.04.2021 >.
// Student: Molete Alexandru-Valentin.
// Grupa: 341 B2.

#include <SimpleDHT.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
LiquidCrystal_PCF8574 lcd(0x27);

const int  buttonPin = 11;    // Pinul la care butonul este atasat.
int counter = 1;              // Contor care numara de cate ori s-a apasat butonul.
int buttonState = 0;          // Starea curenta a butonului.
int lastButtonState = 0;      // Starea anterioara a butonului.
int mode = 0;

// Alegerea pinului pentru masurarea temperaturii.
int pinDHT11 = 2;
SimpleDHT11 dht11(pinDHT11);

// Alegerea pinilor pentru LED-urile de semnalizare a temperaturii dintr- incapere:

const int coldLed = 6;// LED-ul care indica o temperatura scazuta.
const int warmLed = 7;// LED-ul care indica o temperatura placuta.
const int hotLed = 8;// LED-ul care indica o temperatura ridicata.

// Alegerea pinilor pentru reglare.
const int PinRk = A1; // Pinul intrarii analogice la care este atasat potentiometrul.
const int PinYk = A0; // Pinul intrarii analogice la care se masoara marimea procesului.
const int PinUk = 3; // Pinul iesirii digitale la care este atasat LED-ul.

// Parametrii PI:

// Constante
float Te = 0.01; // In secunde.
float Kr = 0.01;
float Ti = 0.001;


// Initializarea variabilelor de reglare:
float rk = 0.0; // Referinta.
float yk = 0.0; // Masura procesului.
float uk = 0.0; // Comanda.
float uk_1 = 0.0; // Comanda anterioara.
float ek = 0.0,ek_1 = 0.0; //Erori.
float q0 = 0.0,q1 = 0.0; // Coeficienti.

// Variabile mecanism Te:
unsigned long nrMil = 0;
unsigned long nrMilPrecedent = 0;

void setup() {
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  
  pinMode(coldLed, OUTPUT);  
  pinMode(warmLed, OUTPUT);    
  pinMode(hotLed, OUTPUT);
  
  // Initializarea comunicarii seriale la 9600 bps:
  Serial.begin(9600);
  
  // Calcularea coeficientilor functiei de transfer, sub forma discreta, a regulatorului:
  q0 = Kr*(1+Te/Ti);
  q1 = -Kr;

  Wire.begin();
  Wire.beginTransmission(0x27);
  int error = Wire.endTransmission();
  if (error == 0) {
    lcd.begin(16, 2); // Initializarea ecranului LCD.
    lcd.setBacklight(255);
    lcd.home(); 
    lcd.clear();
    lcd.print("Loading...");
    delay(1000);
  } else {
    Serial.println(": LCD not found.");
  } // if (error == 0)
} // void setup()

void loop() {
  // Se citeste intrarea pinului conectat la buton:
  buttonState = digitalRead(buttonPin);
   if (buttonState != lastButtonState) {
    // Daca starea a fost schimbata, se incrementeaza contorul.
    if (buttonState == HIGH) {
      // Daca starea curenta este HIGH atunci butonul a fost apasat
      // si se tine cont de cate ori a fost apasat:
      counter++;
    }
    // Se asteapta putin pentru evitarea perturbarii datelor transmise de buton.
    delay(50);
  }
  // Se salveaza starea curenta a butonului ca starea anterioara pentru urmatoarea iteratie a buclei repetitive.
  lastButtonState = buttonState;
  // Se trece la modul de masurare a temperaturii dupa ce a fost apasat de doua ori butonul.
  if (counter >= 3) {
    mode = 1;
    counter = 0;
  }
  // Se trece la modul reglarii nivelului de lumina dupa ce a fost apasat butonul un timp indelungat.
  else if (counter == 1){
    mode=0;
  }
  if(mode == 0)
  {
    Serial.flush(); // Se asteapta ca ultima data de transfer sa fie trimisa. 
    Serial.begin(9600); // Setarea comunicarii seriale la 9600 bps.
    while(Serial.available()) Serial.read();
    
    digitalWrite(coldLed,LOW);
    digitalWrite(hotLed,LOW);
    digitalWrite(warmLed,LOW);
      
    // Implementare mecanism Te.
    nrMil = millis(); // Citirea numarului de milisecunde.
    if((nrMil - nrMilPrecedent) >= (Te*1000))
    {
      nrMilPrecedent = nrMil;// Actualizare variabile mecanism Te.
      
      // Lansare operatii ciclice
      
      int sensorValue = analogRead(PinRk);// Citirea referintei de la potentiometru.
      rk = ((float)sensorValue/1023.0)*100.0;//  Scalare date 0-100%.
      
      sensorValue = analogRead(PinYk);// Citirea masurarii din procesului.
      yk = ((float)sensorValue/1023.0)*100.0;// Scalare date 0-100%.
    
      // Calcul comanda PI (0-100%)
      ek = rk-yk;
      uk = uk_1+q0*ek+q1*ek_1;
      
      // Limitare comanda.
      if(uk > 100.0)
        uk = 100.0;
      if(uk < 0.0)
        uk = 0.0;
        
      // Actualizare memorie algoritm.
      uk_1 = uk;
      ek_1 = ek;
      
      analogWrite(PinUk, (int)((uk/100.0)*255));// Aplicare comanda

      // Afisarea referintei pe LCD:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("rk:");
      lcd.setCursor(3, 0);
      lcd.print(rk);

      // Afisarea iesirii pe LCD:
      lcd.setCursor(8, 0);
      lcd.print("yk:");
      lcd.setCursor(11, 0);
      lcd.print(yk);
  
      // Afisarea comenzii pe LCD:
      lcd.setCursor(0, 1);
      lcd.print("uk:");
      lcd.setCursor(3, 1);
      lcd.print(uk);
    
      // Legenda pentru Serial Plotter:
      Serial.print(rk); // Albastru pentru referinta.
      Serial.print(" ");
      Serial.print(yk); // Rosu pentru iesire.
      Serial.print(" ");
      Serial.println(uk); // Verde pentru comanda.
     } //end if((nrMil - nrMilPrecedent) >= (Te*1000))
  }else
  {
    Serial.flush(); // Se asteapta ca ultima data de transfer sa fie trimisa. 
    Serial.begin(115200); // Setarea comunicarii seriale la 115200 bps.
    while(Serial.available()) Serial.read();
    
    byte temperature = 0;
    byte humidity = 0;
    dht11.read(&temperature,&humidity,NULL);
    temperature = (int)temperature;
    if(temperature < 21 && temperature > 0)
    {
      digitalWrite(coldLed,HIGH);
      digitalWrite(hotLed,LOW);
      digitalWrite(warmLed,LOW);
    }else if(temperature >= 21 && temperature <= 25)
    {
      digitalWrite(coldLed,LOW);
      digitalWrite(hotLed,LOW);
      digitalWrite(warmLed,HIGH);
    }else if(temperature > 25)
    {
      digitalWrite(coldLed,LOW);
      digitalWrite(hotLed,HIGH);
      digitalWrite(warmLed,LOW);
    }

    lcd.begin(16, 2); // initialize the lcd
    lcd.setBacklight(255);
    lcd.home(); 
    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print("Temp:");
    lcd.setCursor(6, 0);
    lcd.print(temperature);
    lcd.setCursor(9, 0);
    lcd.print("*C");
    
    delay(1500);
  } //if(mode == 0)
} // void loop()
