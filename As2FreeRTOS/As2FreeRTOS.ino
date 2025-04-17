  #include <Ticker.h>// to schedule periodic tasks
  #include "freertos/FreeRTOS.h" // freeRTOS needed for assignment
  #include "freertos/task.h"
  #include "freertos/semphr.h"
  #include "B31DGMonitor.h" //monitor task execution 
//define the inputs 
  #define GreenLED     21
  #define RedLED     22
  #define FrequencyLED   19
  #define OnButton     27
  #define ButtonLED  23
  #define Wave1     34
  #define Wave2     33



  B31DGCyclicExecutiveMonitor monitor;
//shared variables between task 3 and 4
  volatile int F1 = 0;
  volatile int F2 = 0;
  volatile bool ledState = false;

  SemaphoreHandle_t buttonSemaphore;//semaphore for button synchronization
  SemaphoreHandle_t freqMutex;//mutex for protecting shared variables

  Ticker tickerSignal1;//ticker for signal 1
  Ticker tickerSignal2;//ticker for signal 2
//generate signal for task 1 using green LED
void TaskA(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        monitor.jobStarted(1);
        
        digitalWrite(GreenLED, HIGH);
        delayMicroseconds(250);
        digitalWrite(GreenLED, LOW);
        delayMicroseconds(50);
        digitalWrite(GreenLED, HIGH);
        delayMicroseconds(300);
        digitalWrite(GreenLED, LOW);

        monitor.jobEnded(1);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));  // 4ms period
    }
}
//generate signal for task 2 using red LED
void TaskB(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        monitor.jobStarted(2);
        
        digitalWrite(RedLED, HIGH);
        delayMicroseconds(100);
        digitalWrite(RedLED, LOW);
        delayMicroseconds(50);
        digitalWrite(RedLED, HIGH);
        delayMicroseconds(200);
        digitalWrite(RedLED, LOW);

        monitor.jobEnded(2);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(3));  // 3ms period
    }
}

//task 3 read the frequency 1
  void TaskC(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      monitor.jobStarted(3);
      unsigned long duration = pulseInLong(Wave1, HIGH, 2000);
      int f = (duration > 0) ? 1000000 / (2 * duration) : 0;
      xSemaphoreTake(freqMutex, portMAX_DELAY);
      F1 = f;
      xSemaphoreGive(freqMutex);
      monitor.jobEnded(3);
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
  }
//task 4 read the frequency 2
  void TaskD(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      monitor.jobStarted(4);
      unsigned long duration = pulseInLong(Wave2, HIGH, 2000);
      int f = (duration > 0) ? 1000000 / (2 * duration) : 0;
      xSemaphoreTake(freqMutex, portMAX_DELAY);
      F2 = f;
      xSemaphoreGive(freqMutex);
      monitor.jobEnded(4);
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
  }
//task 5 do work
  void TaskE(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrequency = pdMS_TO_TICKS(5);
    while (1) {
      monitor.jobStarted(5);
      monitor.doWork();
      monitor.jobEnded(5);
      vTaskDelayUntil(&xLastWakeTime, xfrequency );
    }
  }
//add the frequencies together and display LED
  void TaskF(void *pvParameters) {
    static int lastTotalFreq = 0;
    while (1) {
      xSemaphoreTake(freqMutex, portMAX_DELAY);
      int total = F1 + F2;
      xSemaphoreGive(freqMutex);
      if (total != lastTotalFreq) {  // Only update if frequency changes
        digitalWrite(FrequencyLED, (total > 1500) ? HIGH : LOW);
        lastTotalFreq = total;
      }
      vTaskDelay(pdMS_TO_TICKS(40));
    }
  }
//button turn on LED adn do work 
void TaskG(void *pvParameters) {
    pinMode(OnButton, INPUT_PULLUP);
    bool lastState = digitalRead(OnButton);
    while (1) {
        bool buttonState = digitalRead(OnButton);
        if (buttonState == LOW && lastState == HIGH) {  // Detect button press
            ledState = !ledState;
            digitalWrite(ButtonLED, ledState);
            monitor.doWork();
        }
        lastState = buttonState;
        vTaskDelay(pdMS_TO_TICKS(50));  // Check button every 50 ms
    }
}
//initialize the defiened inputs and ouputs and create the FreeRTOS tasks 
  void setup() {
    Serial.begin(115200);

    pinMode(GreenLED, OUTPUT);
    pinMode(RedLED, OUTPUT);
    pinMode(FrequencyLED, OUTPUT);
    pinMode(ButtonLED, OUTPUT);
    pinMode(Wave1, INPUT);
    pinMode(Wave2, INPUT);

    freqMutex = xSemaphoreCreateMutex(); // create the mutex for the frequency variable
    buttonSemaphore = xSemaphoreCreateBinary();// create the semaphore for the button 

    xTaskCreatePinnedToCore(TaskA, "TaskA", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(TaskB, "Signal2", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(TaskC,   "Freq1",   4096, NULL, 2,   NULL, 1);
    xTaskCreatePinnedToCore(TaskD,   "Freq2",   4096, NULL, 2,   NULL, 1);
    xTaskCreatePinnedToCore(TaskE,  "DoWork",  4096, NULL, 1,   NULL, 0);
    xTaskCreatePinnedToCore(TaskF,     "LED",     4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskG,  "Button",  4096, NULL, 1, NULL, 1);

    monitor.startMonitoring();
  }

  void loop() {
    //empty loop
  }
