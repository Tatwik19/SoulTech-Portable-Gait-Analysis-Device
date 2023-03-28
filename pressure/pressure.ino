int analogPin1 = A0;
int analogPin2= A1;
int analogPin3 = A2;
int analogPin4 = A3;
int analogPin5 = A4;
int analogPin6 = A5;
int analogPin7 = A6;
int analogPin8 = A7; 

// float Vcc = 3.3;
// float sen = 0.003; // Sensitivity
// float v1 = 0, v2 = 0, v3 = 0, v4 = 0, v5 = 0, v6 = 0, v7 = 0, v8 = 0;

int a1 = 0;  // variable to store the value read
int a2 = 0;
int a3 = 0;
int a4 = 0;
int a5 = 0;
int a6 = 0;
int a7 = 0;
int a8 = 0;
float percent = 0;
float sum = 0;
float w1 = 0, w2 = 0, w3 = 0, w4 = 0, w5 = 0, w6 = 0, w7 = 0, w8 = 0;


void setup() {
  Serial.begin(9600);           //  setup serial
}

void loop() {
  pressureCal();
}

void pressureCal(){
  a1 = analogRead(analogPin1);
  a2 = analogRead(analogPin2);
  a3 = analogRead(analogPin3);
  a4 = analogRead(analogPin4);
  a5 = analogRead(analogPin5);
  a6 = analogRead(analogPin6);
  a7 = analogRead(analogPin7);
  a8 = analogRead(analogPin8);
  a1 = 1023 - a1;
  a2 = 1023 - a2;
  a3 = 1023 - a3;
  a4 = 1023 - a4;
  a5 = 1023 - a5;
  a6 = 1023 - a6;
  a7 = 1023 - a7;
  a8 = 1023 - a8;
  percent = 1;
  w1 = a1*percent;
  w2 = a2*percent;
  w3 = a3*percent;
  w4 = a4*percent;
  w5 = a5*percent;
  w6 = a6*percent;
  w7 = a7*percent;
  w8 = a8*percent;
  Serial.print("Analogue Readings:"); 
  Serial.print(w1);
  Serial.print(",\t");
  Serial.print(w2);
  Serial.print(",\t");
  Serial.print(w3);
  Serial.print(",\t");
  Serial.print(w4);
  Serial.print(",\t");
  Serial.print(w5);
  Serial.print(",\t");
  Serial.print(w6);
  Serial.print(",\t");
  Serial.print(w7);
  Serial.print(",\t");
  Serial.println(w8);

  sum = (w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8);
  percent = 100 / sum;
  if(sum < 100){
    percent = 0;
  }
  w1 = w1*percent;
  w2 = w2*percent;
  w3 = w3*percent;
  w4 = w4*percent;
  w5 = w5*percent;
  w6 = w6*percent;
  w7 = w7*percent;
  w8 = w8*percent;
  Serial.print("Percent Readings:"); 
  Serial.print(w1);
  Serial.print(",\t");
  Serial.print(w2);
  Serial.print(",\t");
  Serial.print(w3);
  Serial.print(",\t");
  Serial.print(w4);
  Serial.print(",\t");
  Serial.print(w5);
  Serial.print(",\t");
  Serial.print(w6);
  Serial.print(",\t");
  Serial.print(w7);
  Serial.print(",\t");
  Serial.println(w8);
}