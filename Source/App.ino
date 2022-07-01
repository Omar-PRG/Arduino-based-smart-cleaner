
#include <SoftwareSerial.h>

int rxPin = 10;
int txPin = 8;

SoftwareSerial BTserial(rxPin, txPin);

char Data;
byte trig = 13;
byte echo = 9;
int enA = 2;
int enB = 4;

#define p 25

int in1 = 36;
int in2 = 34;
int in3 = 32;
int in4 = 30;

// Drive 2

byte stopD = 20;
byte maxD = 60;
int enC = 5;
int enD = 6;

int in5 = 45;
int in6 = 41;
int in7 = 43;
int in8 = 39;

bool 
    net = 0 ,
    ASP = 0 ,
    NET = 0 , 
    gau = 0 ,
    doi = 0 ,
    ven = 0 ,
    pp = 0 ,
    F = 0 , 
    B = 0 ,
    D = 0 ,
    G = 0 ;

int vent = 12;
float timeOut = 2 * (maxD + 10) / 100 / 340 * 1000000;
int disti;


void setup(){
    
    delay(200);
    digitalWrite(vent,HIGH);

    BTserial.begin(9600);

    Serial.begin(9600);

    pinMode(enA,OUTPUT);
    pinMode(enB,OUTPUT);
    pinMode(in1,OUTPUT);
    pinMode(in2,OUTPUT);
    pinMode(in3,OUTPUT);
    pinMode(in4,OUTPUT);
    pinMode(vent, OUTPUT);
    
    // Drive 2

    pinMode(enC,OUTPUT);
    pinMode(enD,OUTPUT);
    pinMode(in5,OUTPUT);
    pinMode(in6,OUTPUT);
    pinMode(in7,OUTPUT);
    pinMode(in8,OUTPUT);

    pinMode(p,OUTPUT);
    
    analogWrite(enA,0);
    analogWrite(enB,0);
    analogWrite(enC,0);
    analogWrite(enD,0);
    
    pinMode(trig,OUTPUT);
    pinMode(echo,INPUT);
    
    /* 
    digitalWrite(vent,HIGH);
    delay(4000);
    digitalWrite(vent,LOW);
    delay(4000);
    */
}


void Backward(){

    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW); 

    B = ! B;

    if(B){
        analogWrite(enA,255);
        analogWrite(enB,255);
    } else {
        analogWrite(enA,0);
        analogWrite(enB,0);
    }
}


void droite(){

    D = ! D;
    
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH); 

    if(D){
        analogWrite(enA,0);
        analogWrite(enB,255);
    } else {
        analogWrite(enA,0);
        analogWrite(enB,0);
    }
}


void gauche(){

    G = ! G;
    
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH); 

    if(G){
        analogWrite(enA,255);
        analogWrite(enB,0);
    } else {
        analogWrite(enA,0);
        analogWrite(enB,0);
    }
}



void forward(){

    F = ! F;
    
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH); 

    if(F){
        analogWrite(enA,255);
        analogWrite(enB,255);
        delay(2000);
    } else {
        analogWrite(enA,0);
        analogWrite(enB,0);
    }
}


void pump(){

    pp = ! pp;
    
    if(pp){
        digitalWrite(p,HIGH);  
        delay(180);
    } else {
        digitalWrite(p,LOW);  
    }
}


void Nettoyageauto(){
    
    net = ! net;
    
    if(net){
        digitalWrite(in1,LOW);
        digitalWrite(in2,HIGH);
        digitalWrite(in3,LOW);
        digitalWrite(in4,HIGH); 
        digitalWrite(in5,LOW);
        digitalWrite(in6,HIGH);
        digitalWrite(in7,LOW);
        digitalWrite(in8,HIGH);

        if(Data == 'k'){ 
            
            digitalWrite(vent,LOW);
            
            if(disti > stopD){
                analogWrite(enA,200);
                analogWrite(enB,200);
            } else {
                analogWrite(enA,0);
                analogWrite(enB,0);
                delay(500);
                analogWrite(enB,200);
                delay(1500);
                analogWrite(enA,0);
                analogWrite(enB,0);
            }
        }

        if(Data=='e'){
            
            digitalWrite(vent,HIGH);
            
            if(disti > stopD){
                
                analogWrite(enA,255);
                analogWrite(enB,255);
                
                delay(1000);
                
                analogWrite(enA,0);
                analogWrite(enB,0);
                
                pump();
                
                delay(500);
                
                pump();
                
                // Droite
                
                analogWrite(enA,0);
                analogWrite(enB,255);
                analogWrite(enC,0);
                analogWrite(enD,255);//1
                
                delay(2000);
                
                analogWrite(enD,0);
                analogWrite(enB,0);
                digitalWrite(in1,HIGH);
                digitalWrite(in2,LOW);
                
                delay(500);
                
                analogWrite(enD,255);
                analogWrite(enB,255);//2
                
                delay(2000);
                
                digitalWrite(in1,LOW);
                digitalWrite(in2,HIGH);
                analogWrite(enD,0);
                analogWrite(enB,0);
                
                // Gauche
                
                analogWrite(enA,255);//1
                analogWrite(enB,0);
                analogWrite(enC,255);
                analogWrite(enD,0);
                
                delay(2000);
                
                analogWrite(enC,0);
                analogWrite(enA,0);
                digitalWrite(in3,HIGH);
                digitalWrite(in4,LOW);
                analogWrite(enA,255);//2
                analogWrite(enC,255);
                
                delay(2000);
                
                digitalWrite(in3,LOW);
                digitalWrite(in4,HIGH);
                analogWrite(enC,0);
                analogWrite(enA,0);

            } else {
                analogWrite(enA,0);
                analogWrite(enB,0);
                delay(500);
                analogWrite(enB,255);
                delay(1500);
                analogWrite(enA,0);
                analogWrite(enB,0);
            }
        }
    } else {
        analogWrite(enA,0);
        analogWrite(enB,0);
        analogWrite(enC,0);
        analogWrite(enD,0);
    }
}


void forw(){
     
    NET = ! NET;
    
    digitalWrite(in1, LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH); 
    digitalWrite(in5, LOW);
    digitalWrite(in6,HIGH);
    digitalWrite(in7,LOW);
    digitalWrite(in8,HIGH);

    if(NET){
        analogWrite(enA,255);
        analogWrite(enB,255);
        analogWrite(enC,255);
        analogWrite(enD,255);
    } else {
        analogWrite(enA,0);
        analogWrite(enB,0);
        analogWrite(enC,0);
        analogWrite(enD,0);
    }
}


void Gau(){

    gau = ! gau;
    
    digitalWrite(in1, LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH); 
    digitalWrite(in5, LOW);
    digitalWrite(in6,HIGH);
    digitalWrite(in7,LOW);
    digitalWrite(in8,HIGH);

    if(gau){
        analogWrite(enA,255);
        analogWrite(enB,0);
        analogWrite(enC,255);
        analogWrite(enD,0);
    } else {
        analogWrite(enA,0);
        analogWrite(enB,0);
        analogWrite(enC,0);
        analogWrite(enD,0);
    }
}


void dro(){

    doi = ! doi;
    
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH); 
    digitalWrite(in5,LOW);
    digitalWrite(in6,HIGH);
    digitalWrite(in7,LOW);
    digitalWrite(in8,HIGH);

    if(doi){
        analogWrite(enA,0);
        analogWrite(enB,255);
        analogWrite(enC,0);
        analogWrite(enD,255);
    } else {
        analogWrite(enA,0);
        analogWrite(enB,0);
        analogWrite(enC,0);
        analogWrite(enD,0);
    }
}


void venti(){

    ven = ! ven;
  
    if(ven)
        digitalWrite(vent,LOW);
    else
        digitalWrite(vent,HIGH);
}


int getDistance(){

    unsigned long pulseTime;

    int distance;
    
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig,LOW);
    
    pulseTime = pulseIn(echo,HIGH,timeOut);
    distance = (float) pulseTime * 340 / 2 / 10000;
    
    return distance;
}


void loop(){

    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH); 

    disti = getDistance();
    Serial.println(disti);
    
    /* 
    if(disti > stopD){ 
        analogWrite(enA,255);
        analogWrite(enB,255);
    } else {
        analogWrite(enA,0);
        analogWrite(enB,0);
    }
    */
    
    // The Bluetooth serial port to receive the data in the function
    
    if(BTserial.available()){

        Data = BTserial.read();
        Serial.print(Data);


        digitalWrite(p,LOW); 

        if(Data == 'a'){
            delay(180);
            Serial.print("a");
            forward();
        }

        if(Data == 'b'){
            delay(180);
            Serial.print("b");
            Backward();
        }

        if(Data == 'd'){
            delay(180);
            Serial.print("d");
            gauche();
        }


        if(Data == 'c'){
            delay(180);
            Serial.print("c");
            droite();
        }

        if(Data == 'e'){ 
            Serial.print("e");
            pump();
        }

        if(Data == 'f'){
            delay(180);
            Serial.print("f");
            Nettoyageauto();
        }

        if(Data == 'h'){
            delay(180);
            Serial.print("h");
            forw();
        }

        if(Data == 'i'){
            delay(180);
            Serial.print("i");
            Gau();
        }

        if(Data == 'j'){
            delay(180);
            Serial.print("j");
            dro();
        }
        
        if(Data == 'k')
            venti();
    }
}
