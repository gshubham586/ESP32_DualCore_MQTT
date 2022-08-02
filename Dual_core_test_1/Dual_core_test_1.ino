/*
 * Created by : SHUBHAM GUPTA
 * Message format for digitla pin on/off, node_1/d/, message : 1/1,1/0 or 2/1, 2/0 if using 2nd DO pin and so on.
 * Message format for analog pin o/p (pwm) , node_1/a/, message : 1/analog value (analog value range 0 to 255)
 * Published message : D or A then Pin (1,2...) then Value (0 or 1 in case of Digital, else analog value). Eg. D:1/1 or A:1/655
 * 
 * Fill parameter which are commented "####"
 */



/*
 * INPUT ONLY PINS GPIOs (34, 35, 36, 39)
 * I2C (SDA : 21, SCL : 22)
 * 
 */

 
#include <PubSubClient.h>
#include <WiFi.h>

#define RELAY_1_PIN 19                // Relay 1 pin
#define RELAY_2_PIN 18                // Relay 2 pin
#define RELAY_3_PIN 17                // Realy 3 pin
#define RELAY_4_PIN 16                // Relay 4 Pin
#define ANALOG_1_PIN  23              // PWM pin 
#define DIGITAL_INPUT_PIN_1 35        // Digital Input Pin
#define ANALOG_INPUT_PIN_1 34         // Digital Input Pin
#define PWM1_FREQ 5000                // PWM frequency of PWM pin
#define PWM1_RES 8                    // PWM resolution (2^8 bit)
#define PWM1_CH 0                     // PWM channnel 0
#define DIGITAL_INPUT1_READ_FREQ   30000
#define ANALOG_INPUT1_READ_FREQ    60000

volatile uint8_t RELAY_1_PIN_STATE = HIGH;      // Initial state of Realy 1 pin
volatile uint8_t RELAY_2_PIN_STATE = HIGH;      // Initial state of Realy 2 pin
volatile uint8_t RELAY_3_PIN_STATE = HIGH;      // Initial state of Realy 3 pin
volatile uint8_t RELAY_4_PIN_STATE = HIGH;      // Initial state of Realy 4 pin
volatile uint8_t ANALOG_PIN_VALUE = 0;

volatile uint8_t A_MESSAGE_ARRIVED = 0;         // Flag enabled when Msg for PWM pin is arrived
volatile uint8_t D_MESSAGE_ARRIVED = 0;         // Flag enabled when Msg for Digital Pin is arrived
volatile uint8_t MESSAGE_TO_BE_SENT = 0;        // FLag enabled when there is message to be sent
volatile uint8_t WIFI_SETUP_COMPLETE = 0;       // Flag enabled when Wi-Fi setup is complete

////////////////// MQTT Parameters /////////////////

char wifi_ssid[50] = "SSID";                          // Wi-Fi SSID                    "####"
char wifi_passwd[50] = "Wi-Fi password";              // Wi-Fi password                "####"
int server_port = 1883;                               // MQTT port                     "####"
char mqtt_server_ip[30] = "mqtt broker ip address";   // ip address of mqtt broker     "####"
char mqtt_username[32] = "username";                  // mqtt broker username          "####"
char mqtt_password[32] = "password";                  // mqtt broker password          "####"
char device_name[20] = "node_1";               // device name (Custom name for ESP32, Must be unique if more than 1 nodes are being used in mqtt)
const char sub_topic_1[20] = "node_1/a/";      // Topic 1 to subscribe. You can subscribe multiple topic at a time
const char sub_topic_2[20] = "node_1/d/";      // Topic 2 to subscribe. You can subscribe multiple topic at a time
char pub_topic[20] = "node_1/data";            // Topic name to publish message from node
char dataPublish[150];                         // Variable to store data to be published.
int qos_level = 1; // 0 or 1                   // QoS level of published message
String sub_message = "";                       // Variable to store subscribed message
String pub_message = "";                       // Variable to store message that is to be published
unsigned long current_time = 0;
unsigned long digital_pin_previous_time = 0;
unsigned long analog_pin_previous_time = 0;
TaskHandle_t Task1;
TaskHandle_t Task2;

WiFiClient espClient3;
PubSubClient client(espClient3);

// Callback function. It will be called when any msg will arrive.
void callback(char* topic, byte* payload, unsigned int length)
{
  // Check if topic is subscribed topic 1 or 2
  if(String(topic).substring(0,8) == String(sub_topic_1).substring(0,8))
  {
    Serial.print("Message arrived in topic: ");          
    Serial.println(topic);                               // Print topic
    Serial.print("Message was : ");
    String messageTemp;          
    for (int i = 0; i < length; i++) 
    {
      messageTemp += (char)payload[i];
    }
    sub_message = String(messageTemp);
    A_MESSAGE_ARRIVED = 1;                               // Enable Analog Message arrived flag
    Serial.println(sub_message);                         // Print msg
    
  }

  else if(String(topic).substring(0,8) == String(sub_topic_2).substring(0,8))
  {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);                              // Print topic
    Serial.print("Message was : ");
    String messageTemp;
    for (int i = 0; i < length; i++) 
    {
      messageTemp += (char)payload[i];
    }
    sub_message = String(messageTemp);
    D_MESSAGE_ARRIVED = 1;                             // Enable Digital Message arrived flag
     Serial.println(sub_message);                      // Print msg
  }  
}

// Function to publish mqtt msg
void publishMQTT(char* topics, String data)
{
   data.toCharArray(dataPublish, data.length() + 1);
   client.publish(topics, dataPublish);
}


//Function to connect ESP32 with MQTT broker
void reconnect()
{
  while(!client.connected())
  {
    Serial.print("Connecting to MQTT Server..");
    Serial.println(mqtt_server_ip);
    bool has_connection = client.connect(device_name, mqtt_username, mqtt_password);
    if(has_connection)
    {
      Serial.println("Success connected to MQTT Broker");
      client.subscribe(sub_topic_1, qos_level);            // Subscribe topic 1
      client.subscribe(sub_topic_2, qos_level);            // Subscribe topic 2
    }
    else
    {
      digitalWrite(LED_BUILTIN, HIGH);                
      Serial.print("Failed connected");
      Serial.println(client.state());
      delay(500);
      Serial.println("Trying to connect...");
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  pinMode(RELAY_3_PIN, OUTPUT);
  pinMode(RELAY_4_PIN, OUTPUT);
  pinMode(DIGITAL_INPUT_PIN_1, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  ledcSetup(PWM1_CH, PWM1_FREQ, PWM1_RES);
  ledcAttachPin(ANALOG_1_PIN, PWM1_CH);
  

  digitalWrite(RELAY_1_PIN, RELAY_1_PIN_STATE);
  digitalWrite(RELAY_2_PIN, RELAY_2_PIN_STATE);
  digitalWrite(RELAY_3_PIN, RELAY_3_PIN_STATE);
  digitalWrite(RELAY_4_PIN, RELAY_4_PIN_STATE);
  
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
}

//Core 0 Task:Control GPIOs
void Task1code( void * pvParameters )
{
  for(;;)
  {
    // If wi-fi is setup successfully in core 1
    if(WIFI_SETUP_COMPLETE)
    {
      current_time = millis();
      // If message for analog (pwm) pin is arrived arrived
      if(A_MESSAGE_ARRIVED)
      {
        A_MESSAGE_ARRIVED = 0;                 // Disable message arrived flag
        uint16_t analog_value = atoi(sub_message.substring(2,5).c_str());      // Extract pwm value to write (0 to 255)
        if(analog_value>=0 && analog_value<256)                                // Execute only if pwm value is between 0 to 255
        switch(atoi(sub_message.substring(0,1).c_str()))
        {
        case 1:
          ledcWrite(PWM1_CH, analog_value);
          pub_message = "ANALOG_1_PIN:" + String(analog_value);
          MESSAGE_TO_BE_SENT = 1;
          break;

        default:
          pub_message = "Error_message";
          MESSAGE_TO_BE_SENT = 1;             // Send ACK
        }
        
      }
      // If message for digital pin is arrived
      else if(D_MESSAGE_ARRIVED)
      {
        D_MESSAGE_ARRIVED = 0;                  // Reset digital message arrived flag
        uint8_t NEW_PIN_STATE = LOW;            // Variable to set/reset pin state
        uint8_t state = atoi(sub_message.substring(2,3).c_str());     // Check pin status in msg
        if(state == 1)
        {
          NEW_PIN_STATE = HIGH;
        }
        else if (state == 0)
        {
          NEW_PIN_STATE = LOW;
        }
        // Switch to relay for which the message is came 
        switch(atoi(sub_message.substring(0,1).c_str()))
        {
          case 1:
            digitalWrite(RELAY_1_PIN, NEW_PIN_STATE);           // Change Relay 1 state
            pub_message = "RELAY_1:" + String(state);
            MESSAGE_TO_BE_SENT = 1;
            break;
            
          case 2:
            digitalWrite(RELAY_2_PIN, NEW_PIN_STATE);          // Change Relay 2 state
            pub_message = "RELAY_2:" + String(state);
            MESSAGE_TO_BE_SENT = 1;
            break;
            
          case 3:
            digitalWrite(RELAY_3_PIN, NEW_PIN_STATE);          // Change Relay 3 state
            pub_message = "RELAY_3:" + String(state);
            MESSAGE_TO_BE_SENT = 1;
            break;

          case 4:
            digitalWrite(RELAY_4_PIN, NEW_PIN_STATE);         // Change Relay 4 state
            pub_message = "RELAY_4:" + String(state);
            MESSAGE_TO_BE_SENT = 1;
            break;
          
          default:
          pub_message = "Error_message";
          MESSAGE_TO_BE_SENT = 1;          // Send ACK
        }
    }
    // Raed digital input pin at every "DIGITAL_INPUT1_READ_FREQ" millisec.
    if((current_time - digital_pin_previous_time)>DIGITAL_INPUT1_READ_FREQ)
    {
      pub_message = "D:1/" + String(digital_sensor());
      MESSAGE_TO_BE_SENT = 1;
    }
    // Raed analog input pin at every "ANALOG_INPUT1_READ_FREQ" millisec.
    if((current_time - analog_pin_previous_time)>ANALOG_INPUT1_READ_FREQ)
    {
      pub_message = "A:1/" + String(analog_sensor());
      MESSAGE_TO_BE_SENT = 1;
    }
    vTaskDelay(1);
  }
 }
}

//Control Wi-fi and mqtt
void Task2code( void * pvParameters )
{
   for(;;)
   {
    // If wi-fi setup is not complete or wifi is disconnected
    if((!WIFI_SETUP_COMPLETE) || (WiFi.status() != WL_CONNECTED))
    {
      Serial.print("Connecting to ");
      Serial.println(wifi_ssid);
      
      WiFi.begin(wifi_ssid, wifi_passwd);    // Start wi-fi and connect with given credential
      while (WiFi.status() != WL_CONNECTED) 
      { 
        digitalWrite(LED_BUILTIN, HIGH);                // Blink LED until MCU connect to wi-fi         
        delay(500);
        Serial.print(".");
        digitalWrite(LED_BUILTIN, LOW); 
      }
     Serial.println("");
     Serial.print("WiFi connected - ESP IP address is: ");
     Serial.println(WiFi.localIP());
     client.setServer(mqtt_server_ip,server_port);       // Conncet to mqtt broker
     client.setCallback(callback);                       // Set callback function
     WIFI_SETUP_COMPLETE = 1;
   }
   else if(WiFi.status() == WL_CONNECTED)
    {
      if(!client.connected())
      {
        reconnect();
      }
      client.loop();
      if(MESSAGE_TO_BE_SENT)
      {
        MESSAGE_TO_BE_SENT = 0;
        publishMQTT(pub_topic, pub_message);
      }
    }
   vTaskDelay(1);
   } 
 }

// Read state of digital input sensor
int digital_sensor()
{
  return digitalRead(DIGITAL_INPUT_PIN_1);
}
// Read state of analog input sensor
int analog_sensor()
{
  return analogRead(ANALOG_INPUT_PIN_1);
}

void loop() 
{
delay(1);
}
