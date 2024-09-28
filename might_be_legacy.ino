// TOURBILLON PROJECT - LIBRARIES
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

const char* ssid = "N8C";
const char* password = "wszyscykochajomAlurka1!";
WebServer server(80); // Set web server port number to 80

#define PIN_WS2812B 18  // The ESP32 pin GPIO16 connected to WS2812B
#define NUM_PIXELS 4   // The number of LEDs (pixels) on WS2812B LED strip

Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

// EXP - OLED I2C 
#define SDA_PIN 7
#define SCL_PIN 6
#define SCREEN_WIDTH 128 // [ px ]
#define SCREEN_HEIGHT 64 // [ px ]
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16
static const unsigned char PROGMEM logo_bmp[] = {
  0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000
};

// Define a class to manage serial printing
class SerialPrinter {
public:
    // Constructor to initialize the serial communication
    SerialPrinter() {
        Serial.begin(115200);
    }

    // Function to print the first type of message
    void printMessage1() {
        Serial.println("");
        Serial.println("");
        Serial.println("|-------------------------------------|");
        Serial.println("|-------------------------------------|");
        Serial.println("|       ESP32 GREETS YOU KINDLY!      |");
        Serial.println("|-------------------------------------|");
        Serial.println("|                                     |");
        Serial.println("|             Tourbilon!              |");
        Serial.println("|        Made by: Kamil Dudziak       |");
        Serial.println("|            SOFT REV. 1.0            |");
        Serial.println("|            HARD REV. 1.1            |");
        Serial.println("|                                     |");
        Serial.println("|-------------------------------------|");
        Serial.println("|-------------------------------------|");
        Serial.println("|            INITIALIZATION           |");
        Serial.println("|-------------------------------------|");
        Serial.println("|                                     |");
        Serial.println("|            WiFi CONNECTING          |");
    }

    // Function to print the second type of message
    void printMessage2(const char* localIP) {
        Serial.println(localIP);
        Serial.println("|      mDNS: http://esp32.local       |");
        Serial.println("|                                     |");
        Serial.println("| YOU MUST BE CONNECTED TO SAME WiFi! |");
        Serial.println("|                                     |");
        Serial.println("|-------------------------------------|");
        Serial.println("|                                     |");
        Serial.println("|          HI, IT'S TOURBILON         |");
        Serial.println("|        WE'RE READY TO TAKE OFF!     |");
        Serial.println("|                                     |");
        Serial.println("|-------------------------------------|");
        Serial.println("|                                     |");
        Serial.println("|            WEB SERVER LOGS          |");
        Serial.println("|                 ↓↓↓↓↓               |");
        Serial.println("|                                     |");
    }
};

// Create an instance of SerialPrinter
SerialPrinter oled_printer;

// LedClass using the global ws2812b variable
class LedClass {
public:
    // Constructor to initialize the LED strip
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

// Create an instance of LedClass
LedClass led_controller;

// Define the OLED_Display class to manage the OLED display
class OLED_Display {
public:
    OLED_Display(int width, int height, int reset, int address)
      : screenWidth(width), screenHeight(height), oledReset(reset), screenAddress(address), display(width, height, &Wire, reset) {}

    void begin() {
        Wire.begin(SDA_PIN, SCL_PIN);
        if (!display.begin(SSD1306_SWITCHCAPVCC, screenAddress)) {
            Serial.println(F("SSD1306 allocation failed"));
            for (;;) ; // Don't proceed, loop forever
        }
        display.display();
    }

    void showMessage1(int vb, const char* dvb, const char* rot, const char* sta) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        
        display.setTextSize(2); 
        display.println("TURBINE 1");
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

    void showMessage2(int vb, const char* dvb, const char* rot, const char* sta) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        
        display.setTextSize(2); 
        display.println("TURBINE 2");
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

    void showWebServer(const char* IP) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        
        display.setTextSize(1); 
        display.println("1. Connect to WiFi:");
        display.println("Idz na wybory!");
        display.println("");
        display.println("2. Open web browser");
        display.println("3. Try those URL's");
        display.println("tourbilon.local/?");
        display.println(String(IP) + "/?");
        display.display();
    }

private:
    int screenWidth;
    int screenHeight;
    int oledReset;
    int screenAddress;
    Adafruit_SSD1306 display;
};

// Create an instance of OLED_Display
OLED_Display oled(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_RESET, SCREEN_ADDRESS);

void RTOS_task_alternate_screen(void *parameter) {
    for(;;){
        oled.showMessage1(120, "sin(x)", "CLOCKWISE", "ACTIVE");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        oled.showMessage2(10, "sin(x)", "COUNTERCLOCKWISE", "ACTIVE");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        oled.showWebServer(WiFi.localIP().toString().c_str());
        vTaskDelay(10000 / portTICK_PERIOD_MS);
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

void setup() {
    // SERIAL COMMUNICATION
    oled_printer.printMessage1();

    // OLED I2C
    oled.begin();

    // WEB SERVER
    WiFi.begin(ssid, password); // Connect to Wi-Fi
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
    }
    Serial.println("| WiFi - OK                           |");

    if (!MDNS.begin("esp32")) { // Start the mDNS responder for esp32.local
        Serial.println("| mDNS - FAILED");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("| mDNS - OK                           |");

    if (!SPIFFS.begin(true)) { // Initialize SPIFFS
        Serial.println("| SPIFFS - FAILED");
        SPIFFS.format();
        SPIFFS.begin();
    }
    Serial.println("| SPIFFS - OK                         |");

    // Start server
    webServerStartUp();
    server.begin();

    oled_printer.printMessage2(WiFi.localIP().toString().c_str());

    xTaskCreate(RTOS_task_alternate_screen, "OLED SCREEN TASK", 4096, NULL, 1, NULL);
    xTaskCreate(ROTS_task_led_handler, "LED STRIP TASK", 4096, NULL, 1, NULL);
}

void loop() {
     server.handleClient();
}

// EXP - SETUP WEB SERVER ROUTES
void webServerStartUp(){
    // HOSTING ROUTES
    server.on("/", HTTP_GET, handleRoot);
    server.on("/about", HTTP_GET, handleAbout);

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

// EXP - HANDLE FILE REQUEST
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

// EXP - HANDLE WEB SERVER ROUTES
void handleRoot() {
    handleFileRequest("/main.html", "text/html");
}

void handleAbout() {
    handleFileRequest("/about.html", "text/html");
}

// void testdrawbitmap(void) {
//     display.clearDisplay();

//     display.drawBitmap(
//         (display.width() - LOGO_WIDTH) / 2,
//         (display.height() - LOGO_HEIGHT) / 2,
//         logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1
//     );
//     display.display();
//     delay(1000);
// }