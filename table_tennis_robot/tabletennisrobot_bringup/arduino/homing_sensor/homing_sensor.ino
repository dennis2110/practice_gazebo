uint8_t writecmd[6]={ '$', 0x03,0,0,'\r', '\n'};



void setup() {
  pinMode(13,OUTPUT);
///////serial////////////////////
  Serial.begin(9600);
  while(!Serial);
  Serial.setTimeout(10);
/////////////////////////////////ros
  
////////////cny70////////////////
  pinMode(7,INPUT); // slide rail limit sensor
  pinMode(6,INPUT); // joint 1 limit sensor
/////////////////////////////////
}

void loop() {
////////// read data ////////////
  /*if(Serial.available()){
    size_t sizebyte = Serial.readBytes(readcmd, 5);
    if(readcmd[4]==10){
      for(int i=0;i<4;i++){
        MotorCMD[i] = (int16_t)readcmd[i];
      }
    }else{
      for(int i=0; i<4; i++){
        readcmd[i] = 0;
      }
    }
  }*/
  
/////////////////////////////////

  /*Serial.print(map(analogRead(A0),0,1023,0,255));
  Serial.print("\t");
  Serial.print(map(analogRead(A1),0,1023,0,255));
  Serial.print("\t");
  Serial.print(map(analogRead(A3),0,1023,0,255));
  Serial.print("\t");
  Serial.println(map(analogRead(A4),0,1023,0,255));*/




////////////cny70////////////////


/////////////////////////////////

////////// write data ///////////
//  for(int i=0;i<4;i++){
//    writecmd[i+2] = (uint8_t)MotorCMD[i];
//  }

  if(digitalRead(7)==1){ //joint 0 sensor (1 是未接觸)
    writecmd[2]= 1;
    digitalWrite(13,HIGH);
  }else{
    writecmd[2]= 0;
    digitalWrite(13,LOW);
  }
  if(digitalRead(6)==0){ //joint 1 sensor (0 是未接觸)
    writecmd[3]= 1;
  }else{
    writecmd[3]= 0;
  }
  
  Serial.write(writecmd, 6);
  /*Serial.print("sensor0: ");
  Serial.print(digitalRead(7));
  Serial.print(" ,sensor1: ");
  Serial.println(digitalRead(6));*/
/////////////////////////////////
}
