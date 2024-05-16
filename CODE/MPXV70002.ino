float rho = 1.204; // density of air 


int veloc_mean_size = 20;
int zero_span = 10;
float DifPresure = 0;

// setup and calculate offset
void setup() {
  Serial.begin(115400);
  pinMode(4, INPUT);
}

void loop() {
  float adc_avg = 0; float veloc = 0.0;
  
  for (int ii=0;ii<veloc_mean_size;ii++){
    adc_avg+= analogRead(4);
  }
  adc_avg/=veloc_mean_size;

  if (adc_avg<3125-zero_span){
    DifPresure = ((((adc_avg * 0.0008)/5)-0.5)/0.2)*-1;
    veloc = sqrt(2*(DifPresure*1000)/rho);
  } 
  else{
    Serial.println("ERROR");
  } 
  Serial.print("read ");

  Serial.println(adc_avg);
  Serial.print("speed ");
  
  Serial.println(veloc); // print velocity
  delay(10); // delay for stability
}