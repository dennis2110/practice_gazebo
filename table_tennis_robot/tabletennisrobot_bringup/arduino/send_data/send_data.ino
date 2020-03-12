uint8_t writecmd[6]={ 0x24, 0x41,0x38,0x37,0x25, 0x42};
                     // $,A,8 ,7 ,%,B

void setup() {
  pinMode(13,OUTPUT);
///////serial////////////////////
  Serial.begin(9600);
  while(!Serial);
  Serial.setTimeout(10);
/////////////////////////////////
  
//////////// homing sensor ///////////
  pinMode(7,INPUT); // slide rail limit sensor
  pinMode(6,INPUT); // joint 1 limit sensor
//////////////////////////////////////
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

////////// write data ///////////
  /*if(digitalRead(7)==1){ //joint 0 sensor (1 是未接觸)
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
  }*/
  
  Serial.write(writecmd, 6);
/////////////////////////////////
}
