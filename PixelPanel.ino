#include <Adafruit_Protomatter.h>
#include <Adafruit_LIS3DH.h>    // For accelerometer
#include <HTTPClient.h>
#include <WiFi.h>
#include <Fonts/FreeSansBold12pt7b.h>

#include "FFT_signal.h"
#include "FFT.h"
#include "analog.h"

#define BTN_DOWN 7
#define BTN_UP 6
#define numPoints 64
#define numModes 12

#define MIC_BCK_PIN 3             // Clock pin from the mic.
#define MIC_WS_PIN 9             // WS pin from the mic.
#define MIC_DATA_PIN 11            // Data pin from the mic.
#define MIC_CHANNEL_SELECT_PIN 10 // Pin to select the channel output from the mic. 

#define SAMPLE_SIZE FFT_N
#define SAMPLE_RATE 34910


fft_config_t *real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

uint8_t rgbPins[]  = {42, 41, 40, 38, 39, 37};
uint8_t addrPins[] = {45, 36, 48, 35, 21};
uint8_t clockPin   = 2;
uint8_t latchPin   = 47;
uint8_t oePin      = 14;

Adafruit_Protomatter matrix(
  64,          // Width of matrix (or matrix chain) in pixels
  4,           // Bit depth, 1-6
  1, rgbPins,  // # of matrix chains, array of 6 RGB pins for each
  4, addrPins, // # of address pins (height is inferred), array of pins
  clockPin, latchPin, oePin, // Other matrix control pins
  false);      // No double-buffering here (see "doublebuffer" example)

Adafruit_LIS3DH accel = Adafruit_LIS3DH();
const char* ssid = "Vachon";
const char* password = "51cypress";

#define width 64
#define height 32

long lastCheck = -300000;
String inbound = "None";
String outbound = "None";
int offsetX=0;
String oString = "Loading MBTA information...";

int dMode=5;
long frame=0;

struct Point {
  float x;
  float y;
  float vx;
  float vy;
  uint16_t color;
  uint16_t color2;
};

struct Point points[numPoints];

uint16_t hsv565(float h, float s, float v){
   while(h<0){
    h+=360;
  }
  while(h>360){
    h-=360;
  }
  float c = s*v;
  float hn = h / 60.0;
  float hnm2 = hn;
  while(hnm2>=2){
    hnm2-=2;
  }
  float x = c*(1-abs((hnm2)-1));
  int r=0;
  int g=0;
  int b=0;
  if(hn<1){
    r=c*255;
    g=x*255;
    b=0*255;
    return matrix.color565(r,b,g);
  }
  if(hn<2){
    r=x*255;
    g=c*255;
    b=0*255;
    return matrix.color565(r,b,g);
  }
  if(hn<3){
    r=0*255;
    g=c*255;
    b=x*255;
    return matrix.color565(r,b,g);
  }
  if(hn<4){
    r=0*255;
    g=x*255;
    b=c*255;
    return matrix.color565(r,b,g);
  }
  if(hn<5){
    r=x*255;
    g=0*255;
    b=c*255;
    return matrix.color565(r,b,g);
  }
  if(hn<6){
    r=c*255;
    g=0*255;
    b=x*255;
    return matrix.color565(r,b,g);
  }
  return matrix.color565(64,0,0);
}

const int numAnimFrames=48;
uint16_t animFrames[numAnimFrames][width*height];
int animFrame=0;

void setup() {
  // put your setup code here, to run once:
  fadcInit(1, A1);
  Serial.begin(115200);
  // Initialize matrix...
  ProtomatterStatus status = matrix.begin();
  Serial.print("Protomatter begin() status: ");
  Serial.println((int)status);
  if (!accel.begin(0x19)) {
    Serial.println("Couldn't find accelerometer");
  }
  accel.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  pinMode(BTN_DOWN,INPUT_PULLUP);
  pinMode(BTN_UP,INPUT_PULLUP);

  for(int i=0;i<numPoints;i++){
    points[i].x=i%width;
    points[i].y=i/width;
    points[i].vx = 0;
    points[i].vy = 0;
    points[i].color = hsv565(360*i/numPoints,1,1);
    points[i].color2 = hsv565(360*i/numPoints,1,0.25);
  }

 
}




void liveSat(String view){
  if(millis()-lastCheck>5*60000){
      matrix.fillScreen(matrix.color565(0,64,0));
      matrix.show();
      HTTPClient http;
      http.setTimeout(60000);
      delay(500);
      String serverPath = "http://lucvachon.com/pixelPanelSatService.php?view="+view;
      Serial.println("FEtching...");
      http.begin(serverPath.c_str());
      
      matrix.fillScreen(matrix.color565(0,0,64));
      matrix.show();
      delay(500);
      http.setTimeout(60000);
      int httpResponseCode = http.GET();
      uint8_t byt[6];
      http.getStream().read(byt,4);//Discard first [] bytes (for some reason)
      http.getStream().read(byt,3);//Discard first [] bytes (for some reason)
      for(int frame=0;frame<numAnimFrames;frame++){
        matrix.drawLine(frame,0,frame,height,hsv565(frame*10,1,1));
        matrix.show();
        for(int pixel=0;pixel<width*height;pixel++){
          http.getStream().read(byt,3);
          int r=(byt[0]-48)*4;
          int g=(byt[1]-48)*4;
          int b=(byt[2]-48)*4;
          animFrames[frame][pixel]=matrix.color565(r,b,g);  
        }
        /*if(frame%2){
          http.getStream().read(byt,4);
          //http.getStream().read(byt,4);
        }//Discard [] bytes every other frame (for some reason)
        */
      }
      animFrame=0;
      lastCheck=millis();
  }
  matrix.drawRGBBitmap(0,0,animFrames[animFrame],64,32);
  matrix.show();
  
  
  animFrame=(animFrame+1)%numAnimFrames;
  if(!animFrame){delay(2000);}
  delay(70);
}


void getMBTA(){
  //https://api-v3.mbta.com/predictions?filter[route]=CR-Middleborough&filter[stop]=place-MM-0186&api_key=a5776a64cc384c219bd536a4d1246260&include=trip,vehicle
  int bright=63;
  if(oString==""){
    bright=16;
  }
  matrix.fillScreen(0);
  frame++;
  for(int i=0;i<width;i++){
    //matrix.fillRect(0,0,width,10,matrix.color565(bright,bright*2,0));
    matrix.drawLine(i,0,i,8,hsv565(270.0+10.0*sin(2*3.14159*i/width+frame/30.0),1,bright/63.0));
  }
  matrix.setTextSize(1);
  matrix.setTextWrap(false);
  matrix.setCursor(8, 1);
  matrix.setTextColor(matrix.color565(bright*4,bright*4,bright*4));
  matrix.println("Montello");
  matrix.setTextColor(matrix.color565(bright*4,0,bright*2));
  matrix.setCursor(offsetX,24);
  matrix.setFont(&FreeSansBold12pt7b);
  int16_t  x1, y1;
  uint16_t w, h;
  matrix.getTextBounds(oString, 0, 24, &x1, &y1, &w, &h);
  matrix.setTextSize(1);
  matrix.print(oString);
  matrix.setFont();
  offsetX-=1;
  
  if(-offsetX > w+width){
    offsetX=width;
    if(millis()-lastCheck>1*60000){
      HTTPClient http;
      String serverPath = "http://worldtimeapi.org/api/ip";
      http.begin(serverPath.c_str());
      int httpResponseCode = http.GET();
      String payload = http.getString();
      payload = payload.substring(payload.indexOf("datetime")+11,payload.indexOf("datetime")+30);
      Serial.print("TIME:");
      Serial.println(payload);
      int yr = payload.substring(0,4).toInt();//2023-02-19T17:59:54
      int mo = payload.substring(5,7).toInt();//01234567890123456789
      int da = payload.substring(8,10).toInt();
      int hr = payload.substring(11,13).toInt();
      int mn = payload.substring(14,16).toInt();
      int sc = payload.substring(17,19).toInt();
      Serial.println(String(hr)+":"+String(mn));
      
      HTTPClient http2;
      serverPath = "https://api-v3.mbta.com/predictions?filter[route]=CR-Middleborough&filter[stop]=place-MM-0186&api_key=a5776a64cc384c219bd536a4d1246260&include=trip,vehicle";//"https://api-v3.mbta.com/predictions?filter[route]=Red&filter[stop]=place-wlsta&api_key=a5776a64cc384c219bd536a4d1246260&include=trip,vehicle";
      http2.begin(serverPath.c_str());
      httpResponseCode = http2.GET();
      payload = http2.getString();
      inbound = "None";
      outbound = "None";
      while(payload.indexOf("arrival_time")>0){
           
        payload = payload.substring(payload.indexOf("arrival_time"));
        payload = payload.substring(payload.indexOf("T")+1);
        String temp = payload.substring(0,payload.indexOf("-"));
        int phr = temp.substring(0,2).toInt();
        int pmn = temp.substring(3,5).toInt();
        if(phr<hr){phr+=24;}
        Serial.print("PredTime:");
        Serial.println(temp.substring(3,5));
        Serial.println(String(phr)+":"+String(pmn));
        Serial.println(payload);
        if(payload.indexOf("direction_id\":1")>-1 && payload.indexOf("direction_id\":1")<payload.indexOf("track")){
          Serial.println("Inbound");
          if(inbound=="None"){inbound = String((phr-hr)*60+(pmn-mn))+"mins";}
        }else{
          Serial.println("Outbound");
          if(outbound=="None"){outbound = String((phr-hr)*60+(pmn-mn))+"mins";}
        }
      }
      
      lastCheck=millis();
      oString="";
      if(inbound!="None"){
        oString="Next inbound train in "+inbound;
      }
      if(outbound!="None"){
        if(inbound!="None"){oString+="            ";}
        oString+="Next outbound train in "+outbound;
      }
      
      
    }

    
  }
  matrix.show();
  delay(15);
}

void vapor(){
  matrix.fillScreen(0);
  //Draw sun
  int dia = height/2-4;
  for(int i=dia/2;i<dia;i+=1){
    int w = 0.5*dia*sin(3.14159*i/dia);
    float p = (i-dia/2.0)/(dia/2.0);
    //Mid to bottom
    matrix.drawLine(width/2-w,(i+2),width/2+w,(i+2),hsv565(360-120.0*p,1,1));
    //Mid to top
    matrix.drawLine(width/2-w,(dia+2)-i,width/2+w,(dia+2)-i,hsv565(0+60.0*p,1,1));
  }
  
  //Draw perpindick lines
  for(float x=0;x<width;x++){
    int xx = x+(x-width/2)*10;
    //if(xx<0 || xx>width){continue;}
    matrix.drawLine(x,height/2,xx,height,hsv565(300,0.25,1.0-(abs(width/2-x)/(width/2))));
  }
  
  //Draw horizon lines
  for(float y=height/2+0.1;y<height;y=y+(y-height/2)*1.5){
    matrix.drawLine(0,y,width,y,hsv565(300,1,0.5));
  }
  matrix.show();
  delay(60);
}



int gindex=0;
float lastgx=0;
float lastgy=0;
float lastgz=0;

void gMeter(bool text=false){
  sensors_event_t event;
  accel.getEvent(&event);
  float gx = event.acceleration.x;
  float gy = event.acceleration.y;
  float gz = event.acceleration.z;
  points[gindex].x=gx-lastgx;
  points[gindex].y=gy-lastgy;
  points[gindex].vx=gz-lastgz;
  
  lastgx=lastgx*0.99+gx*0.01;
  lastgy=lastgy*0.99+gy*0.01;
  lastgz=lastgz*0.99+gz*0.01;
  
  gindex=(gindex+1)%64;
  float maxG=0.1;
  for(int i=0;i<64;i++){
    if(abs(points[i].x)>maxG){
      maxG=abs(points[i].x);
    }
    if(abs(points[i].y)>maxG){
      maxG=abs(points[i].y);
    }
    if(abs(points[i].vx)>maxG){
      maxG=abs(points[i].vx);
    }
  }
  maxG = ceil(maxG/1.0)*1.0;
  matrix.fillScreen(0);
  
  uint8_t r[height];
  uint8_t g[height];
  uint8_t b[height];
  
  for(int i=0;i<64;i++){
    int x=(width-gindex+i)%width;
    int yX = (int)(16.0 + 16.0 * points[i].x/maxG);
    int yY = (int)(16.0 + 16.0 * points[i].y/maxG);
    int yZ = (int)(16.0 + 16.0 * points[i].vx/maxG);
    for(int j=0;j<height;j++){
      r[j]=0;
      b[j]=0;
      g[j]=0;
    }
    for(int j=16;j!=yX;){
      r[j]=0+(255*(16-j))/(16-yX);
      if(yX>16){j++;}
      else{j--;}
    }
    for(int j=16;j!=yY;){
      g[j]=0+(255*(16-j))/(16-yY);
      if(yY>16){j++;}
      else{j--;}
    }
    for(int j=16;j!=yZ;){
      b[j]=0+(255*(16-j))/(16-yZ);
      if(yZ>16){j++;}
      else{j--;}
    }
    for(int j=0;j<height;j++){
      matrix.drawPixel(x,j,matrix.color565(r[j],b[j],g[j]));
    }
    //matrix.drawLine(x,yX,x,16,matrix.color565(255,0,0));
    //matrix.drawLine(x,yY,x,16,matrix.color565(0,0,255));
    //matrix.drawLine(x,yZ,x,16,matrix.color565(0,255,0));
  }
  for(int g=0;g<=maxG;g+=5){
    int y = (int)(16.0+16.0*g/maxG);
    matrix.drawPixel(0,y,matrix.color565(255,255,255));
    matrix.drawPixel(width-1,y,matrix.color565(255,255,255));
    matrix.drawPixel(0,32-y,matrix.color565(255,255,255));
    matrix.drawPixel(width-1,32-y,matrix.color565(255,255,255));
  }
  
  
  //Serial.println(maxG);
  if(text){
    matrix.setTextColor(matrix.color565(255,0,0));
    matrix.setCursor(0,0);
    matrix.print(String(gx,1));
    matrix.setTextColor(matrix.color565(0,0,255));
    matrix.setCursor(0,height-8);
    matrix.print(String(gy,1));
    matrix.setTextColor(matrix.color565(0,255,0));
    matrix.setCursor(width-24,0);
    matrix.print(String(gz,1));
    matrix.setTextColor(matrix.color565(255,255,255));
    matrix.setCursor(width-24,height-8);
    matrix.print(String(maxG,1));
  }
  matrix.show();
  delay(15);
}



void balls(){
  sensors_event_t event;
  accel.getEvent(&event);
  matrix.fillScreen(0);
  
  for(int i=0;i<numPoints;i++){
    points[i].vx+=event.acceleration.x/1000;
    points[i].vy+=event.acceleration.y/1000;
   
    points[i].vx-=points[i].vx*0.025;  
    points[i].vy-=points[i].vy*0.025;
        
    points[i].x+=points[i].vx;
    points[i].y+=points[i].vy;
    
    if(points[i].x>=width){
      points[i].x-=width;
      points[i].vx = (random(width*4)-width*2)/100.0;
      points[i].vy = (random(width*4)-width*2)/100.0;
    }
    if(points[i].y>=height){
      points[i].y-=height;
      points[i].vx = (random(width*4)-width*2)/100.0;
      points[i].vy = (random(width*4)-width*2)/100.0;
    }
    if(points[i].x<0){
      points[i].x+=width;
      points[i].vx = (random(width*4)-width*2)/100.0;
      points[i].vy = (random(width*4)-width*2)/100.0;
    }
    if(points[i].y<0){
      points[i].y+=height;
      points[i].vx = (random(width*4)-width*2)/100.0;
      points[i].vy = (random(width*4)-width*2)/100.0;
    }
    matrix.drawPixel((int)(points[i].x),(int)(points[i].y),points[i].color);
  }
  matrix.show();
  delay(15);
  
}




void fluid(){
  sensors_event_t event;
  accel.getEvent(&event);
  matrix.fillScreen(0);
  
  for(int i=0;i<numPoints;i++){
    float ax=0;
    float ay=0;
    for(int j=0;j<numPoints;j++){
      if(j==i){continue;}
      float d = ((points[i].x-points[j].x)*(points[i].x-points[j].x)+(points[i].y-points[j].y)*(points[i].y-points[j].y));
      if(d<0.1){d=0.1;}
      float f = 0.05/d;
      float a = atan2(points[i].y-points[j].y,points[i].x-points[j].x);
      ax+=f*cos(a);
      ay+=f*sin(a);
    }
    
    points[i].vx+=ax+event.acceleration.x/100;
    points[i].vy+=ay+event.acceleration.y/100;
   
    points[i].vx-=points[i].vx*0.025;  
    points[i].vy-=points[i].vy*0.025;
        
    points[i].x+=points[i].vx;
    points[i].y+=points[i].vy;
    
    if(points[i].x>width){
      points[i].x=width-points[i].vx;
      points[i].vx *= -0.99;
    }
    if(points[i].y>height){
      points[i].y=height-points[i].vy;
      points[i].vy *= -0.99;
    }
    if(points[i].x<0){
      points[i].x=0-points[i].vx;
      points[i].vx *= -0.99;
    }
    if(points[i].y<0){
      points[i].y=0-points[i].vy;
      points[i].vy *= -0.99;
    }
    matrix.drawPixel((int)(points[i].x),(int)(points[i].y),points[i].color);
    if(points[i].x>0){
      matrix.drawPixel((int)(points[i].x)-1,(int)(points[i].y),points[i].color2);
    }
    if(points[i].x<width-1){
      matrix.drawPixel((int)(points[i].x)+1,(int)(points[i].y),points[i].color2);
    }
    if(points[i].y>0){
      matrix.drawPixel((int)(points[i].x),(int)(points[i].y)-1,points[i].color2);
    }
    if(points[i].y<height-1){
      matrix.drawPixel((int)(points[i].x),(int)(points[i].y)+1,points[i].color2);
    }
  }
  matrix.show();
  //delay(15);
}


float audioGain = 0.05f;

float audioNF = 1.0f;

float notepow(float n){
  float freq = 55*pow(2,n/12.0);
  float freq2 = 55*pow(2,(n+1.4)/12.0);
  int j = SAMPLE_SIZE*freq/(SAMPLE_RATE);
  int k = SAMPLE_SIZE*freq2/(SAMPLE_RATE);
  //int j = SAMPLE_SIZE*(n)/(NUM_LEDS)/6+1;
  //int k = SAMPLE_SIZE*((n+1))/(NUM_LEDS)/6+1;
  float sum = 0;
  for(int i=j;i<=k;i++){
    float pw = (sqrt(pow(real_fft_plan->output[2*i],2) + pow(real_fft_plan->output[2*i+1],2)));
    sum+=i*pw;
  }
  return audioGain*(sum/((1+k-j))-audioNF);
}



void audioSpectrum(int mo){
  //mic.read(samples);
  
  for (int i = 0 ; i < SAMPLE_SIZE ; i ++) { 
    real_fft_plan->input[i] = (float)(analogReadFast(2)+analogReadFast(2)+analogReadFast(2));
  }
  fft_execute(real_fft_plan); 
  int sumb=0;
  int minPow = 255;
  int maxPow = 0;
  matrix.fillScreen(0);
  for (int i = 0; i < width; i++) {
    int b=(int)notepow((i*1.4));
    if(b<0){b=0;}
    if(b>255){b=255;}
    if(b<minPow){minPow=b;}
    if(b>maxPow){maxPow=b;}
    sumb+=b;
    points[i].y=points[i].y*0.5+b*0.5;
    switch(mo){
      case 0:
        matrix.drawLine(i,height,i,(int)(height-points[i].y/8),hsv565(i*4,1,0.125+points[i].y/300));
        break;
      case 1:
        matrix.drawLine(i,height,i,(int)(height-points[i].y/8),hsv565(360-points[i].y,1,0.125+points[i].y/300.0));
        break;
      case 2:
        for(int j=height-1;j>0;j--){
          animFrames[0][j*width+i]=animFrames[0][(j-1)*width+i];
        }
        animFrames[0][i]=hsv565(360-points[i].y,1,points[i].y/255.0);
        break;
      case 3:
      default:
        if(i%2){continue;}
        for(int j=width-1;j>0;j--){
          animFrames[0][(i/2)*width+j]=animFrames[0][i/2*width+j-1];
        }
        animFrames[0][(i/2)*width]=hsv565(360-points[width-1-i].y,1,points[width-1-i].y/255.0);
        break;
      
    }
  }
  if(mo>=2){
    matrix.drawRGBBitmap(0,0,animFrames[0],64,32);
  }
  matrix.show();
  if(minPow>16 && audioNF<128){
    audioNF*=1.01;
  }
  if(minPow<2 && audioNF>0.01){
    audioNF/=1.01;
  }
  if((sumb/width > 128 || maxPow>=255) && audioGain>0.0001){audioGain*=0.95;Serial.println(String(audioGain,5));}
  if(sumb/width < 32  && audioGain<10.0){audioGain*=1.01;Serial.println(String(audioGain,5));}
  //delay(35);
}

float bigPeak=1025*3;
float bigDC=1024*3;

void audioScope(){
  matrix.fillScreen(0);
  float DC=0;
  float peak=0;
  for(int x=0;x<width;x++){
    animFrames[0][x]=analogReadFast(2)+analogReadFast(2)+analogReadFast(2)+analogReadFast(2)+analogReadFast(2)+analogReadFast(2)+analogReadFast(2)+analogReadFast(2)+analogReadFast(2)+analogReadFast(2)+analogReadFast(2)+analogReadFast(2);
    DC+=animFrames[0][x];
    if(animFrames[0][x]>peak){peak=animFrames[0][x];}
  }
  DC=DC/width;
  bigDC=bigDC*0.99+DC*0.01;
  bigPeak=bigPeak*0.9999;
  if(peak>bigPeak){bigPeak=peak;}
  
  for(int x=0;x<width;x++){
    int y=abs((height/2)*((animFrames[0][x]-bigDC)/(bigPeak-bigDC)));
    if(y>16){y=16;}
    for(int dy=1;dy<=y;dy++){
      if(animFrames[0][x]>DC){
        matrix.drawPixel(x,height/2-dy,hsv565(dy*20,1,1.0*dy/y));
      }else{
        matrix.drawPixel(x,height/2+dy,hsv565(dy*20,1,1.0*dy/y));
      }
    }
  }
  matrix.drawLine(0,height/2,width,height/2,hsv565(0,1,0.125));
  matrix.show();
  delay(15);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  switch(dMode){
    case 0:
      getMBTA();
      break;
    case 1:
      gMeter();
      break;
    case 2:
      gMeter(true);
      break;
    case 3:
      balls();
      break;
    case 4:
      fluid();
      break;
    case 5:
      audioSpectrum(0);
      break;
    case 6:
      audioSpectrum(1);
      break;
    case 7:
      audioSpectrum(2);
      break;
    case 8:
      audioSpectrum(3);
      break;
    case 9:
      audioScope();
      break;
    case 10:
      liveSat("GEOCOLOR");
      break;
    case 11:
      liveSat("Sandwich");
      break;
    default:
      vapor();
  }
  if(!digitalRead(BTN_DOWN)){
    if(dMode>0){
      dMode--;
      lastCheck=millis()-5*60000;
      if(dMode==2 || dMode==1){
        for(int i=0;i<numPoints;i++){
          points[i].x=0;
          points[i].y=0;
          points[i].vx=0;
        }
      }
      delay(500);
    }
  }
  if(!digitalRead(BTN_UP)){
    if(dMode<numModes){
      dMode++;
      lastCheck=millis()-5*60000;
      if(dMode==2 || dMode==1){
        for(int i=0;i<numPoints;i++){
          points[i].x=0;
          points[i].y=0;
          points[i].vx=0;
        }
      }
      delay(500);
    }
  }
  
}
