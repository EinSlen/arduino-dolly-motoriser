#include <SoftwareSerial.h>
#include "Ultrasonic.h"

#define ixPin 11
#define txPin 10

SoftwareSerial bluetooth = SoftwareSerial(ixPin, txPin);
Ultrasonic ultrasonic(7);

#define ROTARY_ANGLE_SENSOR A0
#define ADC_REF 5 //reference voltage of ADC is 5v.If the Vcc switch on the seeeduino
                    //board switches to 3V3, the ADC_REF should be 3.3
#define GROVE_VCC 5 //VCC of the grove interface is normally 5v
#define FULL_ANGLE 300 //full value of the rotary angle is 300 degrees

int MoteurA_sens = 4 ; // Pin 12 Arduino sens Moteur A
int MoteurB_sens = 7 ;  // Pin 13 Arduino sens Moteur B

int ValeurPot=0;  // Variable potentiomètre

int PWM ;  // Variable PWM image de la vitesse
int moteurA_PWM = 5 ;  // Pin 3 Arduino PWM vers MotorShield pour MoteurA
int moteurB_PWM = 6 ;  // Pin 11 Arduino PWM vers MotorShield pour Moteur B

// Code de configuration exécuté une seule fois
void setup() {
Serial.begin(9600);  // Ouverture du port série et debit de communication fixé à 9600 bauds
pinMode(moteurA_PWM, OUTPUT); // Pin 3 Arduino en sortie PWM
pinMode(MoteurA_sens, OUTPUT); // Pin 12 Arduino en sortie digitale
pinMode(ROTARY_ANGLE_SENSOR, INPUT);
digitalWrite (moteurA_PWM,0) ; // au démarrage,  pwm moteurA à zéro
digitalWrite(MoteurA_sens, HIGH) ; // au démarrage, sens moteur A dans sens horaire
digitalWrite (moteurB_PWM,0) ; // au démarrage,  pwm moteurB à zéro
digitalWrite(MoteurB_sens, HIGH) ; // au démarrage, sens moteur B dans sens horaire
pinMode(ixPin, INPUT);
pinMode(txPin, OUTPUT);
bluetooth.begin(9600);

}

// Boucle du programme principal
void loop() {

  if(bluetooth.available()>= 2) {
        unsigned int servopos = bluetooth.read();
        unsigned int servopos1 = bluetooth.read();
        unsigned int realservo = (servopos1 * 256) + servopos;
        PWM = realservo;
        Serial.print("Bluetooth info :");
        Serial.println(realservo);
        bluetooth.flush();
        int RangeIn = ultrasonic.MeasureInInches();
        if(RangeIn < 150) {
        if(PWM>512){
            PWM = PWM-512 ; // on décale la valeur de la plage 512-1024 à 0-512
            digitalWrite(MoteurA_sens,HIGH);  // on commande le moteur dans le sens horaire
            Serial.print("+") ; // on affiche « + » sur le moniteur
            digitalWrite(MoteurB_sens,LOW);  // on commande le moteur dans le sens horaire
            Serial.print("+") ; // on affiche « + » sur le moniteur
            }
            // Sinon on tourne dans l’autre sens
            else { 
               PWM = 512-PWM ; // on décale la valeur de la plage 512-0 à 0-512
              digitalWrite(MoteurA_sens,LOW);  // on commande le moteur dans le sens anti-horaire
              Serial.print("-") ; // on affiche « - » sur le moniteur
              digitalWrite(MoteurB_sens,HIGH);  // on commande le moteur dans le sens anti-horaire
              Serial.print("-") ; // on affiche « - » sur le moniteur
              }
            
            PWM=map(PWM,0,512,0,255);  // convertir la PWM de la plage0-512 à 0-255
            analogWrite(moteurA_PWM,PWM);  // Envoi du signal PWM sur la sortie analogique 3
            analogWrite(moteurB_PWM,PWM);  // Envoi du signal PWM sur la sortie analogique 3
            Serial.print("Valeur PWM : ");  // Affichage sur le moniteur série du texte « Valeur PWM »
            Serial.println(PWM);  // Affichage sur le moniteur série de la valeur PWM
    }
  }
}
