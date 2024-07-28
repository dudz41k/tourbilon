// TOURBILLON PROJECT
// EXP - WEB SERVER
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const char* ssid = "Idz na wybory!";
const char* password = "WszyscykochajomZuzelka!";
WebServer server(80); // Set web server port number to 80

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

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void RTOS_task_alternate_screen(void *parameter) {
    for(;;){
        displayTurbineScreen1(120, "sin(x)", "CLOCKWISE", "ACTIVE");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        displayTurbineScreen2(10, "sin(x)", "COUNTERCLOCKWISE", "ACTIVE");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        displayWebServer(WiFi.localIP().toString().c_str());
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void RTOS_task_serve_server(void *parameter) {
    for(;;){
        server.handleClient();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup() {
    // SERIAL COMMUNICATION
    Serial.begin(115200);
    serialPrinter1();

    // OLED I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    // OLED READY TO WORK 
    display.display();

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

    serialPrinter2(WiFi.localIP().toString().c_str());

    display.clearDisplay();
    testdrawbitmap();
             
    xTaskCreate(RTOS_task_alternate_screen, "OLED SCREEN TASK", 4096, NULL, 1, NULL);
    xTaskCreate(RTOS_task_serve_server, "WEB SERVER TASK", 8192, NULL, 2, NULL);

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

// rot - rotation (CLOCKWISE / COUNTERCLOCKWISE)
// sta - state (ACTIVE / DISABLED)
void displayTurbineScreen1(int vb, const char* dvb, const char* rot, const char* sta){
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE); // Set text color
    display.setCursor(0,0);             // Set cursor position
    
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

// rot - rotation (CLOCKWISE / COUNTERCLOCKWISE)
// sta - state (ACTIVE / DISABLED)
void displayTurbineScreen2(int vb, const char* dvb, const char* rot, const char* sta){
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE); // Set text color
    display.setCursor(0,0);             // Set cursor position
    
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

void displayWebServer(const char* IP){
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE); // Set text color
    display.setCursor(0,0);             // Set cursor position
    
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

// EXP - PRINTER FUNCTION - SERIAL COMMUNICATION 
void serialPrinter1() {
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
    Serial.println("| WiFi CONNECTING                     |");
}

void serialPrinter2(const char* localIP) {
    Serial.println(localIP);
    Serial.println("| mDNS: http://esp32.local            |");
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

void testdrawbitmap(void) {
    display.clearDisplay();

    display.drawBitmap(
        (display.width() - LOGO_WIDTH) / 2,
        (display.height() - LOGO_HEIGHT) / 2,
        logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1
    );
    display.display();
    delay(1000);
}

void neccessary() {
    display.clearDisplay();
}
