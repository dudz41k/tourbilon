// TOURBILLON PROJECT - LIBRARIES
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <AccelStepper.h>
#include <TMC2209.h>

HardwareSerial & serial_stream = Serial1;
const int RX_PIN = 18;
const int TX_PIN = 17;
const int32_t INIT_VELOCITY = 5000;
const uint8_t RUN_CURRENT_PERCENT = 100;

// Instantiate the two TMC2209
TMC2209 stepper_driver_0;
const TMC2209::SerialAddress SERIAL_ADDRESS_0 = TMC2209::SERIAL_ADDRESS_0;
TMC2209 stepper_driver_1;
const TMC2209::SerialAddress SERIAL_ADDRESS_1 = TMC2209::SERIAL_ADDRESS_1;
const uint8_t REPLY_DELAY = 4;
const long SERIAL_BAUD_RATE = 115200;
#define STEP_PIN 9
#define DIR_PIN 10
#define STEP_PIN_2 12
#define DIR_PIN_2 11

AccelStepper stepper1(1, STEP_PIN, DIR_PIN); // (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepper2(1, STEP_PIN_2, DIR_PIN_2);

const int max_speed_na = 1000;
const int max_naccel_na = 1000;

int move_matrix_accelstepper_na[7][3] = {
    {0, max_speed_na, max_naccel_na / 4},
    {3000, max_speed_na, max_naccel_na / 4},
    {0, max_speed_na, max_naccel_na / 4},
    {3000, max_speed_na, max_naccel_na / 2},
    {0, max_speed_na, max_naccel_na / 2},
    {3000, max_speed_na, max_naccel_na},
    {0, max_speed_na, max_naccel_na},
};

const int max_speed_nb = 1000;
const int max_accel_nb = 1000;

int move_matrix_accelstepper_nb[7][3] = {
    {0, max_speed_nb, max_accel_nb / 4},
    {-100, max_speed_nb, max_accel_nb / 4},
    {0, max_speed_nb, max_accel_nb / 4},
    {500, max_speed_nb, max_accel_nb / 2},
    {0, max_speed_nb, max_accel_nb / 2},
    {-1000, max_speed_nb, max_accel_nb},
    {0, max_speed_nb, max_accel_nb},
};

const int max_speed_ma = 1000;
const int max_accel_ma = 1000;

int move_matrix_accelstepper_ma[7][3] = {
    {0, max_speed_ma, max_accel_ma},
    {4000, max_speed_ma, max_accel_ma},
    {8000, max_speed_ma, max_accel_ma},
    {12000, max_speed_ma, max_accel_ma},
    {16000, max_speed_ma, max_accel_ma},
    {20000, max_speed_ma, max_accel_ma},
    {24000, max_speed_ma, max_accel_ma},
};

const int max_speed_mb = 1000;
const int max_accel_mb = 1000;

int move_matrix_accelstepper_mb[7][3] = {
    {0, max_speed_mb, max_accel_mb},
    {-100, max_speed_mb, max_accel_mb},
    {0, max_speed_mb, max_accel_mb},
    {100, max_speed_mb, max_accel_mb},
    {0, max_speed_mb, max_accel_mb},
    {-100, max_speed_mb, max_accel_mb},
    {0, max_speed_mb, max_accel_mb},
};

const char* ssid = "N8C";
const char* password = "wszyscykochajomAlurka1!";


#define PIN_WS2812B 18  // The ESP32 pin GPIO16 connected to WS2812B
#define NUM_PIXELS 4   // The number of LEDs (pixels) on WS2812B LED strip

#define SDA_PIN 7
#define SCL_PIN 6
#define SCREEN_WIDTH 128 // [ px ]
#define SCREEN_HEIGHT 64 // [ px ]
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

unsigned long previous_milis = 0;
QueueHandle_t motorQueueA;
QueueHandle_t motorQueueB;

WebServer server(80);           // Set web server port number to 80
Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

class LedClass {        
public:
    
    LedClass() {
        ws2812b.begin();
        ws2812b.show(); // Initialize all pixels to 'off'
    }

    // Function to turn off all LEDs with delay
    void turn_off_with_delay(int delay_ms) {
        for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
            ws2812b.setPixelColor(pixel, ws2812b.Color(0, 0, 0));
            ws2812b.show();
            vTaskDelay(delay_ms / portTICK_PERIOD_MS); // Delay for the specified time
        }
    }

    // Function to set a pattern with specified colors
    void pattern2(int r, int g, int b) {
        for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
            ws2812b.setPixelColor(pixel, ws2812b.Color(r, g, b));
        }
        ws2812b.show();
    }

    // Function to create a delay
    void delay_ms(int ms) {
        vTaskDelay(ms / portTICK_PERIOD_MS);
    }
};

class SerialPrinter {       
public:
    // Constructor to initialize the serial communication
    SerialPrinter() {
        Serial.begin(SERIAL_BAUD_RATE);
    }

    // Helper function to center-align text within a fixed width
    void printCentered(String text, int width = 50) {
        // int textLength = strlen(text);
        // int textLength = strlen(text);
        int textLength = text.length();
        int padding = (width - textLength) / 2;
        Serial.print("|");
        for (int i = 0; i < padding; i++) {
            Serial.print(" ");
        }
        Serial.print(text);
        for (int i = 0; i < (width - textLength - padding); i++) {  // Ensure balanced padding
            Serial.print(" ");
        }
        Serial.println("|");
    }

    void printDotted(int width = 50) {
        Serial.print("|");
        for (int i=0; i<width; i++){ Serial.print("-"); }
        Serial.println("|");
    }
    
    void printVoid(int width = 50) {
        Serial.print("|");
        for (int i=0; i<width; i++){ Serial.print(" "); }
        Serial.println("|");
    }

    // Function to print the first type of message
    void printMessage1() {
        Serial.println("");
        Serial.println("");
        printDotted();
        printDotted();
        printCentered("ESP32 GREETS YOU KINDLY!");
        printDotted();
        printVoid();
        printCentered("Tourbilon!");
        printCentered("Made by: Kamil Dudziak");
        printCentered("SOFT REV. 1.0");
        printCentered("HARD REV. 1.1");
        printVoid();
        printDotted();
        printDotted();
        printCentered("CALIBRATING TMC2209");
        printCentered("Wait ~ 10s");
        printDotted();
        printDotted();
        printVoid();
    }

    // Function to print the first type of message
    void printMessage1_5() {
        String text_ssid = "Connecting to WiFi: " + String(ssid);
        String text_pass = "With password: " + String(password);

        printDotted();
        printDotted();
        printCentered("Setting up WiFi");
        printDotted();
        printDotted();
        printVoid();
        printCentered("YOUR DEVICE MUST BE CONNECTED TO SAME WiFi!");
        printVoid();
        printCentered(text_ssid);
        printCentered(text_pass);
        printVoid();
        printDotted();
        printDotted();
        printCentered("SETTING UP WEB SERVER");
        printDotted();
        printDotted();
    }



    // Function to print the second type of message
    void printMessage2(const char* localIP) {
        String local_ip_line = "Local IP: " + String(localIP);

        printVoid();
        printCentered("WiFi - OK");
        printCentered("mDNS - OK");
        printCentered("SPIFFS - OK");
        printCentered(local_ip_line);  // Removed the extra '|' print
        printCentered("mDNS: http://esp32.local");
        printVoid();
        printDotted();
        printDotted();
        printVoid();
        printCentered("HI, IT'S TOURBILON");
        printCentered("WE'RE READY TO TAKE OFF!");
        printVoid();
        printDotted();
        printDotted();
        printCentered("↓ WEB SERVER LOGS ↓");
        printDotted();
        printDotted();
    }

   void printMotorStatus(auto& status_motor_a, auto& status_motor_b) {
        printVoid();
        printCentered("[ TMC2209 STATUS: MOTOR A / MOTOR B ]");

        // Print each status field with the format "Motor parameter: value a / value b"
        printCentered(("over_temperature_warning: " + String(status_motor_a.over_temperature_warning) + " / " + String(status_motor_b.over_temperature_warning)).c_str());
        printCentered(("over_temperature_shutdown: " + String(status_motor_a.over_temperature_shutdown) + " / " + String(status_motor_b.over_temperature_shutdown)).c_str());
        printCentered(("short_to_ground_a: " + String(status_motor_a.short_to_ground_a) + " / " + String(status_motor_b.short_to_ground_a)).c_str());
        printCentered(("short_to_ground_b: " + String(status_motor_a.short_to_ground_b) + " / " + String(status_motor_b.short_to_ground_b)).c_str());
        printCentered(("low_side_short_a: " + String(status_motor_a.low_side_short_a) + " / " + String(status_motor_b.low_side_short_a)).c_str());
        printCentered(("low_side_short_b: " + String(status_motor_a.low_side_short_b) + " / " + String(status_motor_b.low_side_short_b)).c_str());
        printCentered(("open_load_a: " + String(status_motor_a.open_load_a) + " / " + String(status_motor_b.open_load_a)).c_str());
        printCentered(("open_load_b: " + String(status_motor_a.open_load_b) + " / " + String(status_motor_b.open_load_b)).c_str());
        printCentered(("over_temperature_120c: " + String(status_motor_a.over_temperature_120c) + " / " + String(status_motor_b.over_temperature_120c)).c_str());
        printCentered(("over_temperature_143c: " + String(status_motor_a.over_temperature_143c) + " / " + String(status_motor_b.over_temperature_143c)).c_str());
        printCentered(("over_temperature_150c: " + String(status_motor_a.over_temperature_150c) + " / " + String(status_motor_b.over_temperature_150c)).c_str());
        printCentered(("over_temperature_157c: " + String(status_motor_a.over_temperature_157c) + " / " + String(status_motor_b.over_temperature_157c)).c_str());
        printCentered(("current_scaling: " + String(status_motor_a.current_scaling) + " / " + String(status_motor_b.current_scaling)).c_str());
        printCentered(("stealth_chop_mode: " + String(status_motor_a.stealth_chop_mode) + " / " + String(status_motor_b.stealth_chop_mode)).c_str());
        printCentered(("standstill: " + String(status_motor_a.standstill) + " / " + String(status_motor_b.standstill)).c_str());

        printVoid();
    }


    void printMotorSettings(auto& settings_motor_a, auto& settings_motor_b) {
        printVoid();
        printCentered("[ TMC2209 SETTINGS: MOTOR A / MOTOR B ]");

        // Print each setting field with the format "Motor parameter: value a / value b"
        printCentered(("is_communicating: " + String(settings_motor_a.is_communicating) + " / " + String(settings_motor_b.is_communicating)).c_str());
        printCentered(("is_setup: " + String(settings_motor_a.is_setup) + " / " + String(settings_motor_b.is_setup)).c_str());
        printCentered(("software_enabled: " + String(settings_motor_a.software_enabled) + " / " + String(settings_motor_b.software_enabled)).c_str());
        printCentered(("microsteps_per_step: " + String(settings_motor_a.microsteps_per_step) + " / " + String(settings_motor_b.microsteps_per_step)).c_str());
        printCentered(("inverse_motor_direction_enabled: " + String(settings_motor_a.inverse_motor_direction_enabled) + " / " + String(settings_motor_b.inverse_motor_direction_enabled)).c_str());
        printCentered(("stealth_chop_enabled: " + String(settings_motor_a.stealth_chop_enabled) + " / " + String(settings_motor_b.stealth_chop_enabled)).c_str());
        // printCentered(("standstill_mode: " + String((settings_motor_a.standstill_mode == TMC2209::NORMAL) ? "normal" :
        //                 (settings_motor_a.standstill_mode == TMC2209::FREEWHEELING) ? "freewheeling" :
        //                 (settings_motor_a.standstill_mode == TMC2209::STRONG_BRAKING) ? "strong_braking" : "braking") + " / " +
        //                 String((settings_motor_b.standstill_mode == TMC2209::NORMAL) ? "normal" :
        //                 (settings_motor_b.standstill_mode == TMC2209::FREEWHEELING) ? "freewheeling" :
        //                 (settings_motor_b.standstill_mode == TMC2209::STRONG_BRAKING) ? "strong_braking" : "braking")).c_str());
        printCentered(("irun_percent: " + String(settings_motor_a.irun_percent) + " / " + String(settings_motor_b.irun_percent)).c_str());
        printCentered(("irun_register_value: " + String(settings_motor_a.irun_register_value) + " / " + String(settings_motor_b.irun_register_value)).c_str());
        printCentered(("ihold_percent: " + String(settings_motor_a.ihold_percent) + " / " + String(settings_motor_b.ihold_percent)).c_str());
        printCentered(("ihold_register_value: " + String(settings_motor_a.ihold_register_value) + " / " + String(settings_motor_b.ihold_register_value)).c_str());
        printCentered(("iholddelay_percent: " + String(settings_motor_a.iholddelay_percent) + " / " + String(settings_motor_b.iholddelay_percent)).c_str());
        printCentered(("iholddelay_register_value: " + String(settings_motor_a.iholddelay_register_value) + " / " + String(settings_motor_b.iholddelay_register_value)).c_str());
        printCentered(("automatic_current_scaling_enabled: " + String(settings_motor_a.automatic_current_scaling_enabled) + " / " + String(settings_motor_b.automatic_current_scaling_enabled)).c_str());
        printCentered(("automatic_gradient_adaptation_enabled: " + String(settings_motor_a.automatic_gradient_adaptation_enabled) + " / " + String(settings_motor_b.automatic_gradient_adaptation_enabled)).c_str());
        printCentered(("pwm_offset: " + String(settings_motor_a.pwm_offset) + " / " + String(settings_motor_b.pwm_offset)).c_str());
        printCentered(("pwm_gradient: " + String(settings_motor_a.pwm_gradient) + " / " + String(settings_motor_b.pwm_gradient)).c_str());
        printCentered(("cool_step_enabled: " + String(settings_motor_a.cool_step_enabled) + " / " + String(settings_motor_b.cool_step_enabled)).c_str());
        printCentered(("analog_current_scaling_enabled: " + String(settings_motor_a.analog_current_scaling_enabled) + " / " + String(settings_motor_b.analog_current_scaling_enabled)).c_str());
        printCentered(("internal_sense_resistors_enabled: " + String(settings_motor_a.internal_sense_resistors_enabled) + " / " + String(settings_motor_b.internal_sense_resistors_enabled)).c_str());

        printVoid();
        printDotted();
    }
};

class OledClass {
public:
    OledClass() {}

    // `vb` - stepper rot. velocity  [int] [si_unit: RPM]
    // `dvb` - derrivative of motor rot. velocity [c.char/str] [SIN(A), COS(A), SIN(2A)...] 
    // `rot` - rotation [c.char/str] [CLOCKWISE, COUNTERCLOCKWISE]
    // `sta` - state [c.char/str] [ACTIVE, DISABLED]
    void displayScrenTurbineA(int vb, const char* dvb, const char* rot, const char* sta){
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE); 
        display.setCursor(0,0);            
        display.setTextSize(2); 
        display.println("TURBINE A");
        display.setTextSize(1); 
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.println(sta);
        display.setTextColor(SSD1306_WHITE);
        display.println("");
        display.println(String(vb) + " RPM");
        display.println("dV ~ " + String(dvb) + " m/s2");
        display.println(rot);
        display.display();
    }
    
    // `vb` - stepper rot. velocity  [int] [si_unit: RPM]
    // `dvb` - derrivative of motor rot. velocity [c.char/str] [SIN(A), COS(A), SIN(2A)...] 
    // `rot` - rotation [c.char/str] [CLOCKWISE, COUNTERCLOCKWISE]
    // `sta` - state [c.char/str] [ACTIVE, DISABLED]
    void displayScrenTurbineB(int vb, const char* dvb, const char* rot, const char* sta){
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE); 
        display.setCursor(0,0);             
        display.setTextSize(2); 
        display.println("TURBINE B");
        display.setTextSize(1); 
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.println(sta);
        display.setTextColor(SSD1306_WHITE);
        display.println("");
        display.println(String(vb) + " RPM");
        display.println("dV ~ " + String(dvb) + " m/s2");
        display.println(rot);
        display.display();
    }

    void displayWiFiScreen(const char* IP){
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE); 
        display.setCursor(0,0);             
        display.setTextSize(1); 
        display.println("1. Connect to WiFi:");
        display.println("Idz na wybory!");
        display.println("");
        display.println("2. Open web broswer");
        display.println("3. Try those URL's");
        display.println("tourbilon.local/?");
        display.println(String(IP) + "/?");
        display.display();
    }
};

class StepperClass {
  public:
    StepperClass() {}

    // Method to move motor A using the matrix
    void move_a(int move_matrix[][3], int rows) {
        for (int i = 0; i < rows; i++) {
            int target_position = move_matrix[i][0];
            int max_speed = move_matrix[i][1];
            int accel = move_matrix[i][2];

            stepper1.setMaxSpeed(max_speed);
            stepper1.setAcceleration(accel);
            stepper1.moveTo(target_position);

            Serial.print("(A) x: ");
            Serial.print(target_position);
            Serial.print(" v: ");
            Serial.print(max_speed);
            Serial.print(" a: ");
            Serial.println(accel);

            while (stepper1.currentPosition() != target_position) {
                stepper1.run();
                vTaskDelay(1);
            }
        }
    }

    // Method to move motor B using the matrix
    void move_b(int move_matrix[][3], int rows) {
        for (int i = 0; i < rows; i++) {
            int target_position = move_matrix[i][0];
            int max_speed = move_matrix[i][1];
            int accel = move_matrix[i][2];

            stepper2.setMaxSpeed(max_speed);
            stepper2.setAcceleration(accel);
            stepper2.moveTo(target_position);

            Serial.print("(B) x: ");
            Serial.print(target_position);
            Serial.print(" v: ");
            Serial.print(max_speed);
            Serial.print(" a: ");
            Serial.println(accel);

            while (stepper2.currentPosition() != target_position) {
                stepper2.run();
                vTaskDelay(1);
            }
        }
    }
};


LedClass led_controller;        
SerialPrinter serial_printer;     
StepperClass stepper_controller;
OledClass oled_controller;


struct MotorCommand {
  String motor;
  String matrix;
};

void stepperTask(void *parameter) {
    MotorCommand command;

    while (true) {
        // Wait for a command from either motorQueueA or motorQueueB
        if (xQueueReceive(motorQueueA, &command, 0)) {
            if (command.matrix == "M1") {
                stepper_controller.move_a(move_matrix_accelstepper_na, 7);
            } else if (command.matrix == "M2") {
                stepper_controller.move_a(move_matrix_accelstepper_ma, 7);
            }else if (command.matrix == "M3") {
                stepper_controller.move_a(move_matrix_accelstepper_mb, 7);
            }else if (command.matrix == "M4") {
                stepper_controller.move_a(move_matrix_accelstepper_mb, 7);
            }
        }

        if (xQueueReceive(motorQueueB, &command, 0)) {
            if (command.matrix == "M1") {
                stepper_controller.move_b(move_matrix_accelstepper_nb, 7);
            } else if (command.matrix == "M2") {
                stepper_controller.move_b(move_matrix_accelstepper_mb, 7);
            }else if (command.matrix == "M3") {
                stepper_controller.move_b(move_matrix_accelstepper_ma, 7);
            }else if (command.matrix == "M4") {
                stepper_controller.move_b(move_matrix_accelstepper_ma, 7);
            }
        }

        vTaskDelay(1);  // To avoid blocking
    }
}

void RTOS_task_alternate_screen(void *parameter) {
    for(;;){
        // displayTurbineScreen1(120, "sin(x)", "CLOCKWISE", "ACTIVE");
        oled_controller.displayScrenTurbineA(120, "sin(x)", "CLOCKWISE", "ACTIVE");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        oled_controller.displayScrenTurbineB(10, "sin(x)", "CLOCKWISE", "ACTIVE");
        // displayTurbineScreen2(10, "sin(x)", "COUNTERCLOCKWISE", "ACTIVE");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        oled_controller.displayWiFiScreen(WiFi.localIP().toString().c_str());
        // displayWebServer(WiFi.localIP().toString().c_str());
        // vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void ROTS_task_led_handler(void *parameter){
    for(;;){
        led_controller.pattern2(0, 100, 0);
        led_controller.delay_ms(5000); // 5-second delay
        led_controller.turn_off_with_delay(1000); // Turn off LEDs one by one with 1-second delay
        led_controller.delay_ms(1000); // 1-second delay
    }
}

void ROTS_task_stepper_motor_A(void *parameter){
    MotorCommand command;
    while (true) {
        if (xQueueReceive(motorQueueA, &command, 0)) {
            if (command.matrix == "M1") { stepper_controller.move_a(move_matrix_accelstepper_na, 7); } 
            else if (command.matrix == "M2") { stepper_controller.move_a(move_matrix_accelstepper_ma, 7); }
            else if (command.matrix == "M3") { stepper_controller.move_a(move_matrix_accelstepper_mb, 7); }
            else if (command.matrix == "M4") { stepper_controller.move_a(move_matrix_accelstepper_mb, 7); }
        }
        vTaskDelay(1);  
    }
}

void ROTS_task_stepper_motor_B(void *parameter){
    MotorCommand command;
    while (true) {
        if (xQueueReceive(motorQueueB, &command, 0)) {
            if (command.matrix == "M1") { stepper_controller.move_b(move_matrix_accelstepper_na, 7); } 
            else if (command.matrix == "M2") { stepper_controller.move_b(move_matrix_accelstepper_ma, 7); }
            else if (command.matrix == "M3") { stepper_controller.move_b(move_matrix_accelstepper_mb, 7); }
            else if (command.matrix == "M4") { stepper_controller.move_b(move_matrix_accelstepper_mb, 7); }
        }
        vTaskDelay(1);  
    }
}

void setup_sequence_TMC2209(){
    stepper_driver_0.setup(serial_stream,SERIAL_BAUD_RATE,SERIAL_ADDRESS_0, RX_PIN, TX_PIN);
    stepper_driver_0.setReplyDelay(REPLY_DELAY);
    stepper_driver_1.setup(serial_stream,SERIAL_BAUD_RATE,SERIAL_ADDRESS_1, RX_PIN, TX_PIN);
    stepper_driver_1.setReplyDelay(REPLY_DELAY);

    stepper_driver_0.setRunCurrent(RUN_CURRENT_PERCENT);
    stepper_driver_0.setMicrostepsPerStep(16);
    stepper_driver_0.disableAutomaticCurrentScaling();
    stepper_driver_0.disableAutomaticGradientAdaptation();
    stepper_driver_0.disableInverseMotorDirection();
    stepper_driver_0.setPwmOffset(64);
    // stepper_driver_0.setPwmGradient(128);
    // stepper_driver_0.enableCoolStep();

    stepper_driver_1.setRunCurrent(RUN_CURRENT_PERCENT);
    stepper_driver_1.setMicrostepsPerStep(16);
    stepper_driver_1.disableAutomaticCurrentScaling();
    stepper_driver_1.disableAutomaticGradientAdaptation();
    stepper_driver_1.enableInverseMotorDirection();
    stepper_driver_1.setPwmOffset(64);
    // stepper_driver_1.setPwmGradient(128);
    // stepper_driver_1.enableCoolStep();
    
    stepper_driver_0.enable();
    stepper_driver_1.enable();
    delay(1000);

    // stepper_driver_0.moveAtVelocity(0);
    // stepper_driver_1.moveAtVelocity(0);
    // delay(3000);
    
    // stepper_driver_0.moveAtVelocity(INIT_VELOCITY);
    // stepper_driver_1.moveAtVelocity(INIT_VELOCITY);
    // delay(3000);
    
    // stepper_driver_0.moveAtVelocity(-INIT_VELOCITY);
    // stepper_driver_1.moveAtVelocity(INIT_VELOCITY);
    // delay(3000);
    
    // stepper_driver_0.moveAtVelocity(-INIT_VELOCITY);
    // stepper_driver_1.moveAtVelocity(-INIT_VELOCITY);
    // delay(3000);

    TMC2209::Settings settings_motor_a = stepper_driver_0.getSettings();
    TMC2209::Settings settings_motor_b = stepper_driver_1.getSettings();
    TMC2209::Status status_motor_a = stepper_driver_0.getStatus();
    TMC2209::Status status_motor_b = stepper_driver_1.getStatus();
    serial_printer.printMotorSettings(settings_motor_a, settings_motor_b);
    serial_printer.printMotorStatus(status_motor_a, status_motor_b);

    delay(1000);

    stepper_driver_0.moveAtVelocity(0);
    stepper_driver_1.moveAtVelocity(0);
}
 
void setup() {
    serial_printer.printMessage1();

    // VIRTUAL SETUP FOR TMC2209 VIA UART 
    setup_sequence_TMC2209();

    serial_printer.printMessage1_5();

    // SETUP OLED I2C DISPLAY
    Wire.begin(SDA_PIN, SCL_PIN);
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { Serial.println(F("SSD1306 allocation failed")); for(;;); }
    display.display();

    // SETUP WEB SERVER
    WiFi.begin(ssid, password); 
    while (WiFi.status() != WL_CONNECTED) { delay(1000); }
    if (!MDNS.begin("esp32")) { Serial.println("| mDNS - FAILED"); while (1) {delay(1000);} }
    if (!SPIFFS.begin(true)) { Serial.println("| SPIFFS - FAILED"); SPIFFS.format(); SPIFFS.begin(); }
    webServerStartUp();
    server.begin();

    serial_printer.printMessage2(WiFi.localIP().toString().c_str());

    delay(1000);

    motorQueueA = xQueueCreate(5, sizeof(MotorCommand));
    motorQueueB = xQueueCreate(5, sizeof(MotorCommand));

    // CREATE RTOS TASKS FOR ESP32 FOR SECOND CORE (CORE 0 - FIRST CORE - BY DEFAULT IS RESERVED FOR HANDLING WEB SERVER)
    // xTaskCreatePinnedToCore(stepperTask, "Stepper Task", 8192, NULL, 1, NULL, 1);  // Run on Core 1
    xTaskCreate(RTOS_task_alternate_screen, "OLED SCREEN TASK", 4096, NULL, 1, NULL);
    xTaskCreate(ROTS_task_led_handler, "LED STRIP TASK", 4096, NULL, 1, NULL);
    xTaskCreatePinnedToCore(ROTS_task_stepper_motor_A, "STEPPER A MOTOR TASK", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(ROTS_task_stepper_motor_B, "STEPPER B MOTOR TASK", 8192, NULL, 1, NULL, 1);
    // xTaskCreate(ROTS_task_stepper_motor_A, "STEPPER A MOTOR TASK", 8192, NULL, 1, NULL);
    // xTaskCreate(ROTS_task_stepper_motor_B, "STEPPER B MOTOR TASK", 8192, NULL, 1, NULL);

}

void loop() {
    server.handleClient();
}

void webServerStartUp(){
    // HOSTING ROUTES
    server.on("/set", HTTP_GET, handleSetParameters);
    server.on("/settings", HTTP_GET, handleSetSettings);
    
    // server.on("/", HTTP_GET, handleRoot);
    // server.on("/about", HTTP_GET, handleAbout);
    server.on("/", HTTP_GET, []() {
        handleFileRequest("/main.html", "text/html");
    });
    server.on("/about", HTTP_GET, []() {
        handleFileRequest("/about.html", "text/html");
    });
    server.on("/control", HTTP_GET, []() {
        handleFileRequest("/control.html", "text/html");
    });

    // HOSTING STATIC FILES
    server.on("/styles.css", HTTP_GET, []() {
        handleFileRequest("/styles.css", "text/css");
    });

    server.on("/scripts.js", HTTP_GET, []() {
        handleFileRequest("/scripts.js", "application/javascript");
    });

    server.on("/bcd", HTTP_GET, []() {
        handleFileRequest("/bcd.png", "image/png");
    });

    server.onNotFound([]() {
        Serial.print("| error: 404 - FILE NOT FOUND - UNKNOWN");
        server.send(404, "text/plain", "File not found");
    });
}

void handleFileRequest(const char* path, const char* mimeType) {
    File file = SPIFFS.open(path, "r");
    if (!file) {
        Serial.printf("| error: 404 - FILE NOT FOUND - %s\n", path);
        server.send(404, "text/plain", "File not found");
        return;
    }
    Serial.printf("| file: %s\n", path);
    server.streamFile(file, mimeType);
    file.close();
}

void handleSetParameters(){
    MotorCommand command;
    command.motor = server.arg("motor");
    command.matrix = server.arg("matrix");
    
    if (command.motor == "A") { xQueueSend(motorQueueA, &command, portMAX_DELAY); } 
    else if (command.motor == "B") { xQueueSend(motorQueueB, &command, portMAX_DELAY); }
    
    handleFileRequest("/set.html", "text/html");
}

void handleSetSettings(){
    MotorCommand command;
    command.motor = server.arg("motor");
    command.matrix = server.arg("matrix");

    // xQueueSend(motorQueue, &command, portMAX_DELAY);

    // Serial.print("Motor parameter: " + motor + "Matrix: " + matrix );
    
    // if (motor == "A") {
    //     if (matrix == "matrix1") {stepper_controller.move_a(move_matrix_accelstepper_na, 3);} 
    //     else if (matrix == "matrix2") {stepper_controller.move_a(move_matrix_accelstepper_ma, 3);}
    // } else if (motor == "B") {
    //     if (matrix == "matrix1") {stepper_controller.move_b(move_matrix_accelstepper_nb, 3);} 
    //     else if (matrix == "matrix2") {stepper_controller.move_b(move_matrix_accelstepper_mb, 3);} 
    // }

    // Send a basic response back to the client
    server.send(200, "text/plain", "Motor " + command.motor + " set to " + command.matrix);
}

// // Initialize GPIO pins
    // pinMode(STEP_PIN, OUTPUT);
    // pinMode(DIR_PIN, OUTPUT);
    // digitalWrite(DIR_PIN, HIGH); 
    // for (int i = 0; i < 8000; i++) {
    //     // Generate a step pulse
    //     digitalWrite(STEP_PIN, HIGH);  // Step pulse ON
    //     delayMicroseconds(5);          // Pulse width (adjust for motor driver specs)
    //     digitalWrite(STEP_PIN, LOW);   // Step pulse OFF
    //     Serial.println("Moving!");

    //     // Add a delay between steps to control speed (adjust for motor driver)
    //     delay(5);  // Delay in milliseconds (adjust this to control speed)
    // }


