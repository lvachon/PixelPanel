#include <Adafruit_Protomatter.h>
#include <Adafruit_LIS3DH.h>    // For accelerometer

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

#define numPoints 64

float xCoords[numPoints];// = {0,0,0,0,0};
float yCoords[numPoints];// = {0,0,0,0,0};
float xVels[numPoints];// = {0,0,0,0,0};
float yVels[numPoints];// = {0,0,0,0,0};

#define width 64
#define height 32

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
  for(int i=0;i<numPoints;i++){
    xCoords[i]=random(width);
    yCoords[i]=random(height);
    xVels[i] = (random(width*10)-width*5)/100.0;
    yVels[i] = (random(width*10)-width*5)/100.0;
  }
}

uint16_t hsv565(float h, float s, float v){
  float c = s*v;
  float hn = h / 60.0;
  float hnm2 = hn;
  while(hnm2>2){
    hnm2-=2;
  }
  while(h<0){
    h+=360;
  }
  while(h>360){
    h-=360;
  }
  float x = c*(1-abs((hnm2)-1));
  int r=0;
  int g=0;
  int b=0;
  if(hn<1){
    r=c*255;
    g=x*255;
    b=0*255;
    return matrix.color565(r,g,b);
  }
  if(hn<2){
    r=x*255;
    g=c*255;
    b=0*255;
    return matrix.color565(r,g,b);
  }
  if(hn<3){
    r=0*255;
    g=c*255;
    b=x*255;
    return matrix.color565(r,g,b);
  }
  if(hn<4){
    r=0*255;
    g=x*255;
    b=c*255;
    return matrix.color565(r,g,b);
  }
  if(hn<5){
    r=x*255;
    g=0*255;
    b=c*255;
    return matrix.color565(r,g,b);
  }
  if(hn<6){
    r=c*255;
    g=0*255;
    b=x*255;
    return matrix.color565(r,g,b);
  }
  return matrix.color565(64,0,0);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t event;
  accel.getEvent(&event);

  for(int i=0;i<numPoints;i++){
    matrix.drawPixel((int)(xCoords[i]),(int)(yCoords[i]),hsv565(0,0,0));
  }
  
  for(int i=0;i<numPoints;i++){
    xVels[i]+=event.acceleration.x/1000;
    yVels[i]+=event.acceleration.y/1000;
    if(true || (xVels[i]*xVels[i]+yVels[i]*yVels[i])>0.5){
      xVels[i]-=xVels[i]*0.01;  
      yVels[i]-=yVels[i]*0.01;
    }
    
    
    
    xCoords[i]+=xVels[i];
    yCoords[i]+=yVels[i];
    
    if(xCoords[i]>=width){
      xCoords[i]-=width;
      xVels[i] = (random(width*10)-width*5)/100.0;
      yVels[i] = (random(width*10)-width*5)/100.0;
    }
    if(yCoords[i]>=height){
      yCoords[i]-=height;
      xVels[i] = (random(width*10)-width*5)/100.0;
      yVels[i] = (random(width*10)-width*5)/100.0;
    }
    if(xCoords[i]<0){
      xCoords[i]+=width;
      xVels[i] = (random(width*10)-width*5)/100.0;
      yVels[i] = (random(width*10)-width*5)/100.0;
    }
    if(yCoords[i]<0){
      yCoords[i]+=height;
      xVels[i] = (random(width*10)-width*5)/100.0;
      yVels[i] = (random(width*10)-width*5)/100.0;
    }
    matrix.drawPixel((int)(xCoords[i]),(int)(yCoords[i]),hsv565(i*360/numPoints,1.0,1.0));
  }
  matrix.show();
  Serial.println(matrix.getFrameCount());
  delay(15);
}
