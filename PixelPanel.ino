#include <Adafruit_Protomatter.h>
#include <Adafruit_LIS3DH.h>    // For accelerometer
#include <HTTPClient.h>
#include <WiFi.h>
#include <Fonts/FreeSansBold12pt7b.h>

#define BTN_DOWN 7
#define BTN_UP 6
#define numPoints 64
#define numModes 5

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


float xCoords[numPoints];// = {0,0,0,0,0};
float yCoords[numPoints];// = {0,0,0,0,0};
float xVels[numPoints];// = {0,0,0,0,0};
float yVels[numPoints];// = {0,0,0,0,0};


void setup() {
  // put your setup code here, to run once:
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
    xCoords[i]=random(width);
    yCoords[i]=random(height);
    xVels[i] = (random(width*10)-width*5)/100.0;
    yVels[i] = (random(width*10)-width*5)/100.0;
  }
  
}

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

long lastCheck = -150000;
String inbound = "None";
String outbound = "None";
int offsetX=0;
String oString = "Loading MBTA information...";

int dMode=3;
long frame=0;

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


float gxs[64];
float gys[64];
float gzs[64];
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
  gxs[gindex]=gx-lastgx;
  gys[gindex]=gy-lastgy;
  gzs[gindex]=gz-lastgz;
  
  lastgx=lastgx*0.99+gx*0.01;
  lastgy=lastgy*0.99+gy*0.01;
  lastgz=lastgz*0.99+gz*0.01;
  
  gindex=(gindex+1)%64;
  float maxG=0.1;
  for(int i=0;i<64;i++){
    if(abs(gxs[i])>maxG){
      maxG=abs(gxs[i]);
    }
    if(abs(gys[i])>maxG){
      maxG=abs(gys[i]);
    }
    if(abs(gzs[i])>maxG){
      maxG=abs(gzs[i]);
    }
  }
  maxG = ceil(maxG/1.0)*1.0;
  matrix.fillScreen(0);
  
  uint8_t r[height];
  uint8_t g[height];
  uint8_t b[height];
  
  for(int i=0;i<64;i++){
    int x=(width-gindex+i)%width;
    int yX = (int)(16.0 + 16.0 * gxs[i]/maxG);
    int yY = (int)(16.0 + 16.0 * gys[i]/maxG);
    int yZ = (int)(16.0 + 16.0 * gzs[i]/maxG);
    for(int j=0;j<height;j++){
      r[j]=0;
      b[j]=0;
      g[j]=0;
    }
    for(int j=16;j!=yX;){
      r[j]=32+(223*(16-j))/(16-yX);
      if(yX>16){j++;}
      else{j--;}
    }
    for(int j=16;j!=yY;){
      g[j]=32+(223*(16-j))/(16-yY);
      if(yY>16){j++;}
      else{j--;}
    }
    for(int j=16;j!=yZ;){
      b[j]=32+(223*(16-j))/(16-yZ);
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
    matrix.drawPixel((int)(xCoords[i]),(int)(yCoords[i]),hsv565(0,0,0));
  }
  
  for(int i=0;i<numPoints;i++){
    xVels[i]+=event.acceleration.x/1000;
    yVels[i]+=event.acceleration.y/1000;
    if(true || (xVels[i]*xVels[i]+yVels[i]*yVels[i])>0.5){
      xVels[i]-=xVels[i]*0.025;  
      yVels[i]-=yVels[i]*0.025;
    }
    
    xCoords[i]+=xVels[i];
    yCoords[i]+=yVels[i];
    
    if(xCoords[i]>=width){
      xCoords[i]-=width;
      xVels[i] = (random(width*4)-width*2)/100.0;
      yVels[i] = (random(width*4)-width*2)/100.0;
    }
    if(yCoords[i]>=height){
      yCoords[i]-=height;
      xVels[i] = (random(width*4)-width*2)/100.0;
      yVels[i] = (random(width*4)-width*2)/100.0;
    }
    if(xCoords[i]<0){
      xCoords[i]+=width;
      xVels[i] = (random(width*4)-width*2)/100.0;
      yVels[i] = (random(width*4)-width*2)/100.0;
    }
    if(yCoords[i]<0){
      yCoords[i]+=height;
      xVels[i] = (random(width*4)-width*2)/100.0;
      yVels[i] = (random(width*4)-width*2)/100.0;
    }
    matrix.drawPixel((int)(xCoords[i]),(int)(yCoords[i]),hsv565(i*360/numPoints,1.0,1.0));
  }
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
    default:
      vapor();
  }
  if(!digitalRead(BTN_DOWN)){
    if(dMode>0){
      dMode--;
      delay(500);
    }
  }
  if(!digitalRead(BTN_UP)){
    if(dMode<numModes){
      dMode++;
      delay(500);
    }
  }
  
}
