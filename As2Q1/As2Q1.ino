#include <B31DGMonitor.h>//library for monitoring
#include <Ticker.h>//used to implement the major cycle 
B31DGCyclicExecutiveMonitor monitor;
Ticker tickytack;
//define the pins based on the inputs and outputs used
int GreenLED = 21;
int RedLED = 22;
int YellowLED = 23;
int FrequencyLED = 19;
int OnButton = 27;
int inputF1 = 34;
int inputF2 = 33;
//variables for the button
bool ledState = false;   // LED state (on/off)
bool lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

//tracking variables for the task timing
unsigned long lastTask1Time = 0;
unsigned long lastTask2Time = 0;
unsigned long lastTask3Time = 0;
unsigned long lastTask4Time = 0;
unsigned long lastTask5Time = 0;

unsigned long currentTime = micros();
unsigned long F1;
unsigned long F2;

void setup() {
 // initialization of the inputs and ouputs of hardware and monitoring 
 Serial.begin(115200);
 pinMode(GreenLED, OUTPUT);
 pinMode(RedLED, OUTPUT);
 pinMode(YellowLED, OUTPUT);
 pinMode(OnButton, INPUT);
 pinMode(inputF1, INPUT);
 pinMode(inputF2, INPUT);
 monitor.startMonitoring();
 tickytack.attach_ms(1, allTasks);//cyclic executive scheduler


}
//generate the first signal for task 1 using green LED
void taskA() {
  monitor.jobStarted(1);
  digitalWrite(GreenLED, HIGH);
  delayMicroseconds(250);
  digitalWrite(GreenLED, LOW);
  delayMicroseconds(50);
  digitalWrite(GreenLED, HIGH);
  delayMicroseconds(300);
  digitalWrite(GreenLED, LOW);
  monitor.jobEnded(1);
}
//generate the second signal for task 2 using red LED
void taskB(){
  monitor.jobStarted(2);
  digitalWrite(RedLED, HIGH);
  delayMicroseconds(100);
  digitalWrite(RedLED, LOW);
  delayMicroseconds(50);
  digitalWrite(RedLED, HIGH);
  delayMicroseconds(200);
  digitalWrite(RedLED, LOW);
  monitor.jobEnded(2);
}
//task 3 measuring the frequency from F1 input signal 
void taskC(){
  monitor.jobStarted(3);
  int currentstate = digitalRead(inputF1);
  static int previousstate = LOW;
  static bool edge = false;
  unsigned long period = 0; 
  unsigned long prevtime = 0;

  if (currentstate == HIGH && previousstate == LOW){
    if (edge){
      period = currentTime - prevtime;
      F1 = 1000000.0/period;
    }
    prevtime = currentTime;
    edge = true;
  }
previousstate = currentstate;
monitor.jobEnded(3);
}
//measuring the frequency of f2 input segnal 
void taskD(){
  monitor.jobStarted(4);
  int currentstate2 = digitalRead(inputF2);
  static int previousstate2 = LOW;
  static bool edge2 = false;
  unsigned long period2 = 0;
  unsigned long prevtime2 = 0;

  if (currentstate2 == HIGH && previousstate2 == LOW){
    if (edge2){
      period2 = currentTime - prevtime2;
      F2 = 1000000.0/period2;

    }
    prevtime2 = currentTime;
    edge2 = true;
    
  }
  previousstate2 = currentstate2;
  monitor.jobEnded(4);
}
//task 5 just call the function doWork (500 microseconds)
void taskE(){
  monitor.jobStarted(5);
  monitor.doWork();
  monitor.jobEnded(5);
}
//cyclic executive scheduler, ensures that the tasks run based on priority 
void allTasks(){
    currentTime = micros(); // Get the current time once
    while (currentTime - lastTask2Time >= 3000){
      taskB();
      lastTask2Time = currentTime;
    }    
    while (currentTime - lastTask1Time >= 4000){
      taskA();
      lastTask1Time = currentTime;
    }
    while (currentTime - lastTask3Time >= 10000){
      taskC();
      lastTask3Time = currentTime;
    }
    while (currentTime - lastTask4Time >= 10000){
      taskD();
      lastTask4Time = currentTime;
    }
    while (currentTime - lastTask5Time >= 5000){
      taskE();
      lastTask5Time = currentTime;
    }
}
// handle button press and toggle LED
void loop() {
 int buttonState = digitalRead(OnButton);
        if (buttonState == LOW && lastButtonState == HIGH){
          if (millis() - lastDebounceTime > debounceDelay){
            ledState = !ledState;
            digitalWrite(YellowLED,ledState);
            monitor.doWork();
            lastDebounceTime = millis();
          }
        }
        lastButtonState = buttonState;

  
}

