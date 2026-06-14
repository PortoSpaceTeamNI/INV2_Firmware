#include <Arduino.h>
#include<ADS1115_WE.h> 
#include<Wire.h>

#define I2C_ADDRESS 0x48

volatile int interruptPin = 36;
int ledPin = 2;
volatile bool outOfLimit = false;
volatile bool convReady = false;

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

void outOfLimitAlert();
void convReadyAlert();
void single_mode_stm();

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);
  Serial2.begin(115200);
  
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  if(!adc.init()){
    Serial.println("ADS1115 not connected!");
  }

  /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   * 
   * ADS1115_RANGE_6144  ->  +/- 6144 mV
   * ADS1115_RANGE_4096  ->  +/- 4096 mV
   * ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1115_RANGE_1024  ->  +/- 1024 mV
   * ADS1115_RANGE_0512  ->  +/- 512 mV
   * ADS1115_RANGE_0256  ->  +/- 256 mV
   */
  adc.setVoltageRange_mV(ADS1115_RANGE_6144); //comment line/change parameter to change range

  /* Set the inputs to be compared
   *  
   *  ADS1115_COMP_0_1    ->  compares 0 with 1 (default)
   *  ADS1115_COMP_0_3    ->  compares 0 with 3
   *  ADS1115_COMP_1_3    ->  compares 1 with 3
   *  ADS1115_COMP_2_3    ->  compares 2 with 3
   *  ADS1115_COMP_0_GND  ->  compares 0 with GND
   *  ADS1115_COMP_1_GND  ->  compares 1 with GND
   *  ADS1115_COMP_2_GND  ->  compares 2 with GND
   *  ADS1115_COMP_3_GND  ->  compares 3 with GND
   */
  adc.setCompareChannels(ADS1115_COMP_0_GND); //comment line/change parameter to change range

  /* Set number of conversions out of limit after which alert pin asserts
   * - or you can disable the alert (including conversion ready alert)
   *  
   *  ADS1115_ASSERT_AFTER_1  -> after 1 conversion
   *  ADS1115_ASSERT_AFTER_2  -> after 2 conversions
   *  ADS1115_ASSERT_AFTER_4  -> after 4 conversions
   *  ADS1115_DISABLE_ALERT   -> disable comparator / alert pin (default) 
   */
  adc.setAlertPinMode(ADS1115_ASSERT_AFTER_4); // alternative: ...AFTER_2 or 4. If you disable this sketch does not work

  /* Set the conversion rate in SPS (samples per second)
   * Options should be self-explaining: 
   * 
   *  ADS1115_8_SPS 
   *  ADS1115_16_SPS  
   *  ADS1115_32_SPS 
   *  ADS1115_64_SPS  
   *  ADS1115_128_SPS (default)
   *  ADS1115_250_SPS 
   *  ADS1115_475_SPS 
   *  ADS1115_860_SPS 
   */
  adc.setConvRate(ADS1115_860_SPS); //uncomment if you want to change the default

  /* Set continuous or single shot mode:
   * 
   *  ADS1115_CONTINUOUS  ->  continuous mode
   *  ADS1115_SINGLE      ->  single shot mode (default)
   */
  //adc.setMeasureMode(ADS1115_CONTINUOUS); //comment or change you want to change to single shot
  adc.setMeasureMode(ADS1115_SINGLE); //comment or change you want to change to single shot


  /* Sets the alert pin polarity if active:
   *  
   * ADS1115_ACT_LOW  ->  active low (default)   
   * ADS1115_ACT_HIGH ->  active high
   */
  //adc.setAlertPol(ADS1115_ACT_LOW); //uncomment if you want to change the default
  
  /* With this function the alert pin assert, when a conversion is ready.
   * In order to deactivate, use the setAlertLimit_V function  
   */
  adc.setAlertPinToConversionReady(); //needed for this sketch
  
  Serial.println("ADS1115 Example Sketch - Single Shot, Conversion Ready Alert Pin controlled");
  Serial.println();
  //attachInterrupt(digitalPinToInterrupt(interruptPin), convReadyAlert, FALLING);
  //adc.startSingleMeasurement();
}

void loop()
{
  single_mode_stm();
}

//void loop() {
  //static int counter = 1;
  //static mux arr[] = {ADS1115_COMP_0_GND, ADS1115_COMP_1_GND, ADS1115_COMP_2_GND, ADS1115_COMP_3_GND};
  
  //if(convReady){
    //convReady = false;
    
    //float voltage = 0.0;
    //voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
    ////delay(5); 
    //Serial2.println(counter);
    
    //adc.setCompareChannels(arr[counter++]); //comment line/change parameter to change range
    //counter = counter % 2;

    ////if(counter == 0) delay(5);

    ////adc.startSingleMeasurement();   
  //}
//}

#define TIMEOUT 2
#define SAMPLE_TIME 50
void single_mode_stm()
{
  static uint8_t state = 0;
  static uint8_t counter = 0;

  static mux arr[] = {ADS1115_COMP_0_GND, ADS1115_COMP_1_GND, ADS1115_COMP_2_GND, ADS1115_COMP_3_GND};

  static unsigned long time; 
  static unsigned long time0;

  switch (state)
  {
  case 0: //start new cycle of readings
  {
    //take time before the readings start
    time0 = millis();
  }
  //no break, it is intentional
  case 1: //set channel for reading
  {
    Serial2.print("counter: ");
    Serial2.println(counter);

    digitalWrite(12, HIGH);

    adc.setCompareChannels(arr[counter++]); //comment line/change parameter to change range
    counter = counter % 2;

    digitalWrite(12, LOW);

    adc.startSingleMeasurement();
    
    digitalWrite(12, HIGH);
    
    time = millis();

    Serial2.println(state);

    state = 2;
  }
  break;

  case 2: //wait for reading
  {
    digitalWrite(12, LOW);

    if(millis() - time > TIMEOUT) state = 3;
  }
  break;
  
  case 3: //get reading
  {
    if(adc.isBusy()) break;
    
    digitalWrite(12, HIGH);

    float voltage = 0.0;
    voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
    
    digitalWrite(12, LOW);

    Serial2.println(state);

    //time = millis();
    if(counter == 0) state = 4; 
    else state = 1;
  }
  break;

  case 4: //wait for next cycle of readings
  {
    if(millis() - time0 > SAMPLE_TIME) state = 0;
  }

  default:
    break;
  }
}

void convReadyAlert(){
  convReady = true;
}
//void loop() {
  //int16_t value = ADS.readADC(0);
  ////Serial.printf("V: %d\n", value);
  //Serial2.printf("0\n");
  //delay(2);
  //value = ADS.readADC(1);
  ////Serial.printf("V: %d\n", value);
  //Serial2.printf("1\n");
  //delay(2);
  //value = ADS.readADC(2);
  ////Serial.printf("V: %d\n", value);
  //Serial2.printf("2\n");
  //delay(2);
  
  ////value = ADS.readADC(3);
  //////Serial.printf("V: %d\n", value);
  ////Serial2.println("3");
  ////delay(2);
//}
