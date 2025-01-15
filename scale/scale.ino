int reading;
int offset;
int user;
float input[20];
float avg = 0;
float output;


void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  pinMode(10, INPUT);
  offset = 444;
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("Enter 'z' to tare, or enter to read weight");
  
  while (Serial.available() == 0){}
  if(Serial.available() > 0){
    user = Serial.read();
  }
  if(user == 'z'){
    offset = avg;
  }

  for(int x = 0; x<20; x++){
    input[x] = analogRead(A0);
    avg = avg + input[x];
  }
  avg = avg/20.0;
  reading = avg - offset;
  output = mass(reading);
  Serial.print("Weight in g: ");
  Serial.println(output);
  Serial.println(reading);
  delay(10);
}

float mass(float x){
  float weight;
  if(reading <= 0){
    
    weight = 0;
  }
  else if(reading < 230){    //if item is less than 100g (not in the linear region)
    weight = 10.0*log(x);
  }
  else{
    weight = 0.0147*pow(x, 2) - 8.9337*x + 1455.1;
  }
  
  return weight;
}
