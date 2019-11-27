uint8_t writecmd[5]={ '$', 0x03,0,'\r', '\n'};



void setup() {
  pinMode(13,OUTPUT);
///////serial////////////////////
  Serial.begin(115200);
  while(!Serial);
  Serial.setTimeout(10);
/////////////////////////////////ros
  
////////////cny70////////////////
  pinMode(6,INPUT);
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

  if(digitalRead(7)==1){
    writecmd[2]= 1;
    digitalWrite(13,HIGH);
  }else{
    writecmd[2]= 0;
    digitalWrite(13,LOW);
  }
  Serial.write(writecmd, 5);
/////////////////////////////////
}
