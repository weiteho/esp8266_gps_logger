#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SD_CS 2  // Use a safe pin (e.g., GPIO2)

const unsigned long updateInterval = 1000; //one second. 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

WiFiServer telnetServer(23);
WiFiClient telnetClient;

TinyGPSPlus gps;
File gpsFile;
File csvFile;
String lastFileName;

unsigned long lastScreenUpdate;

int bytesRead;
String dataReceived = "";
String currentLine = "";

byte gpsBaud115200[] = {
  0xB5, 0x62,             // UBX header
  0x06, 0x00,             // CFG-PRT
  0x14, 0x00,             // Length = 20
  0x01, 0x00,             // PortID = 1 (UART1), reserved
  0x00, 0x00,             // txReady = 0
  0xD0, 0x08, 0x00, 0x00, // baudRate = 115200 (little endian)
  0x00, 0x00,             // inProtoMask = UBX+NMEA
  0x08, 0x00,             // outProtoMask = UBX
  0x00, 0x00,             // flags, reserved
  0x00, 0x00,             // reserved
  0xA2, 0xB5              // checksum
};

byte gps10Hz[] = {
  0xB5, 0x62, 0x06, 0x08, // UBX-CFG-RATE header
  0x06, 0x00,             // payload length
  0x64, 0x00,             // measRate = 100ms (0x0064)
  0x01, 0x00,             // navRate = 1
  0x01, 0x00,             // timeRef = UTC
  0x7A, 0x12              // checksum
};

byte gps5Hz[] = {
  0xB5, 0x62,       // UBX header
  0x06, 0x08,       // CFG-RATE
  0x06, 0x00,       // Payload length
  0xC8, 0x00,       // measRate = 200ms (0x00C8)
  0x01, 0x00,       // navRate = 1
  0x01, 0x00,       // timeRef = 0 = UTC
  0xDE, 0x6A        // checksum (automatically correct for above)
};

byte gps1Hz[] = {
  0xB5, 0x62, 0x06, 0x08,
  0x06, 0x00,
  0xE8, 0x03,             // 1000 ms = 1 Hz (0x03E8)
  0x01, 0x00,
  0x01, 0x00,
  0x01, 0x39
};

// Web server and WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/gps");

unsigned long lastBroadcast = 0;
// Flag to set time only once
bool localTimeSet = false;

bool sdAvailable = false;

#define WIFI_CONNECT_TIMEOUT 10000  // 10 seconds per SSID attempt

struct WiFiCredential {
  String ssid;
  String password;
};

std::vector<WiFiCredential> wifiList;

void readWiFiListFromSD() {
  if (!SD.begin(SD_CS)) {
    // Serial.println("SD card failed to initialize. Skipping WiFi file.");
    return;
  }

  File configFile = SD.open("/wifi.txt");
  if (!configFile) {
    // Serial.println("wifi.txt not found. Skipping.");
    return;
  }

  while (configFile.available()) {
    String line = configFile.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("#")) continue;

    int sepIndex = line.indexOf(' ');
    if (sepIndex == -1) continue;

    String ssid = line.substring(0, sepIndex);
    String password = line.substring(sepIndex + 1);
    ssid.trim();
    password.trim();

    if (ssid.length() > 0) {
      wifiList.push_back({ ssid, password });
    }
  }

  configFile.close();
}

bool tryConnectToWiFi(const WiFiCredential& cred) {
  // Serial.printf("Trying SSID: %s\n", cred.ssid.c_str());
  WiFi.begin(cred.ssid.c_str(), cred.password.c_str());

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < WIFI_CONNECT_TIMEOUT) {
    delay(500);
    display.print(".");
    display.display();
  }

  return WiFi.status() == WL_CONNECTED;
}

void connectToWiFiFromList() {
  for (const auto& cred : wifiList) {
    if (tryConnectToWiFi(cred)) {
      display.print("\nWiFi connected. IP: ");
      display.println(WiFi.localIP());
      return;
    }
  }
}

void sendGpsData() {
  if (gps.location.isValid()) {
    String json = "{";
    json += "\"lat\":" + String(gps.location.lat(), 6) + ",";
    json += "\"lng\":" + String(gps.location.lng(), 6) + ",";
    json += "\"time\":\"" + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "\",";
    json += "\"date\":\"" + String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day()) + "\"";
    json += "}";
    ws.textAll(json);
  }
}

// Convert GPS date/time to Unix timestamp
time_t getGPSTime() {
  if (gps.date.isValid() && gps.time.isValid()) {
    struct tm t;
    t.tm_year = gps.date.year() - 1900;
    t.tm_mon  = gps.date.month() - 1;
    t.tm_mday = gps.date.day();
    t.tm_hour = gps.time.hour();
    t.tm_min  = gps.time.minute();
    t.tm_sec  = gps.time.second();
    t.tm_isdst = 0;
    return mktime(&t);
  }
  return 0;
}

String getTimestampForFileName() {
  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char name[14];
  snprintf(name, sizeof(name), "%d%02d%02d_%02d", (t->tm_year + 1900), (t->tm_mon + 1), t->tm_mday, t->tm_hour);
  return String(name);
}

String getFolderPath() {
  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char name[11];
  snprintf(name, sizeof(name), "%d/%02d/%02d", (t->tm_year + 1900), (t->tm_mon + 1), t->tm_mday);
  return String(name);
}


void setup() {
  //display  
  Wire.begin(4, 5);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Init GPS Logger");
  display.display();
  //wifi
  display.print("Connecting to WIFI");
  display.display();
  readWiFiListFromSD();
  connectToWiFiFromList();
  //telnet server
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  //web socket server
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      // Serial.printf("WebSocket client #%u connected\n", client->id());
    }
  });
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", R"rawliteral(
      <!DOCTYPE html><html>
      <head><title>GPS Tracker</title></head>
      <body>
        <h2>GPS Info</h2>
        <p id="data">Waiting for GPS...</p>
        <script>
          var ws = new WebSocket("ws://" + location.host + "/gps");
          ws.onmessage = function(event) {
            var data = JSON.parse(event.data);
            document.getElementById("data").innerHTML =
              "Date: " + data.date + "<br>" +
              "Time: " + data.time + "<br>" +
              "Latitude: " + data.lat + "<br>" +
              "Longitude: " + data.lng + "<br>";
          };
        </script>
        <hr>
        <a href="list">Files</a>
      </body></html>
    )rawliteral");
  });
  // Route: /list?dir=/subfolder
  server.on("/list", HTTP_GET, [](AsyncWebServerRequest *request) {
    String dirPath = "/";
    int start = 0;
    int limit = 10;
    if (request->hasParam("dir")) {
      dirPath = request->getParam("dir")->value();
    }
    if (request->hasParam("start")) start = request->getParam("start")->value().toInt();
    if (request->hasParam("limit")) limit = request->getParam("limit")->value().toInt();

    File root = SD.open(dirPath.c_str(), "r");
    if (!root || !root.isDirectory()) {
      request->send(500, "text/plain", "Failed to open directory");
      return;
    }

    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->print("<h2>Files in: " + dirPath + "</h2><ul>");

    File file = root.openNextFile();
    int index = 0, count = 0;

    while (file && count < limit) {
      if (index >= start && count < limit) {
        response->print("<li>");
        if (file.isDirectory()) {
          response->print("<a href='/list?dir=" + dirPath + (dirPath.endsWith("/") ? "" : "/") + file.name() + "'>");
          response->print(file.name());
          response->print("/");
          response->print("</a>");
        } else if (file.size() > 0) {
          response->print("<a href='/download?file=" + dirPath + (dirPath.endsWith("/") ? "" : "/") + file.name() + "'>");
          response->print(file.name());
          response->print("</a>");
          response->print(" (" + String(file.size()) + " bytes)");
        }
        else {
          response->print(file.name());
          response->print(" (" + String(file.size()) + " bytes)");
        }
        response->print("</li>");
        count++;
      }
      index++;
      file = root.openNextFile();
    }
    response->print("</ul>");
    response->print("<p>Showing " + String(start+1) + " to " + String(start + count) + "</p>");
    // Pagination buttons
    int nextStart = start + count;
    int prevStart = max(start - limit, 0);

    response->print("<div style='margin-top:10px;'>");

    if (start > 0) {
      response->print("<a href=\"/list?dir=" + dirPath + "&start=" + String(prevStart) + "&limit=" + String(limit) + "\">");
      response->print("<button>&laquo; Previous</button></a> ");
    }

    if (count == limit) {
      response->print("<a href=\"/list?dir=" + dirPath + "&start=" + String(nextStart) + "&limit=" + String(limit) + "\">");
      response->print("<button>Next &raquo;</button></a>");
    }

    response->print("</div>");

    request->send(response);
  });
  //download file
  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("file")) {
      request->send(400, "text/plain", "Missing 'file' parameter");
      return;
    }

    String path = request->getParam("file")->value();

    if (!path.startsWith("/")) {
      request->send(400, "text/plain", "Invalid file path");
      return;
    }

    File file = SD.open(path, FILE_READ);
    if (!file || file.isDirectory()) {
      request->send(404, "text/plain", "File not found");
      return;
    }

    size_t fileSize = file.size();
    File* filePtr = new File(file); // Needed because the chunk function must capture a pointer

    AsyncWebServerResponse *response = request->beginChunkedResponse("application/octet-stream",
      [filePtr](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        if (!filePtr->available()) {
          filePtr->close();
          delete filePtr;
          return 0; // End of file
        }

        return filePtr->read(buffer, maxLen); // Read chunk into buffer
      }
    );

    response->addHeader("Content-Disposition", "attachment; filename=\"" + String(filePtr->name()) + "\"");
    response->addHeader("Cache-Control", "no-cache");

    request->send(response);
  });

  server.begin();

  // Start GPS Serial (hardware)
  Serial.begin(9600);  // GPS TX is connected to GPIO3 (RX)
  // // set baud rate to 115200
  // Serial.write(gpsBaud115200, sizeof(gpsBaud115200));
  // delay(200);
  // Serial.end();
  // Serial.begin(115200);  // GPS TX is connected to GPIO3 (RX)
  // delay(200);

  // // Serial.write(gps10Hz, sizeof(gps10Hz));
  // // Serial.write(gps5Hz, sizeof(gps5Hz));
  // Serial.write(gps1Hz, sizeof(gps1Hz));
  // delay(200);

  if (Serial.available()) {
    display.println("GPS Ready");
  }

  // OTA setup
  ArduinoOTA.setHostname("esp8266-ota"); // Optional custom hostname
  ArduinoOTA.begin();

    // Start SD
  if (!SD.begin(SD_CS)) {
    sdAvailable = false;
    display.println("SD Init Fail");
  }
  else {
    sdAvailable = true;
    display.println("SD Ready");
  }
  display.display();
  // Init time (approximate)
  setTime(0);

  lastScreenUpdate = millis();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    //handle OTA
    ArduinoOTA.handle();
  }
  // Handle Telnet client
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
    } else {
      telnetServer.available().stop(); // Only one client at a time
    }
  }

  unsigned long now = millis();
  String strNow = String(now);

  //read GPS data
  while (Serial.available()) {
    char c = Serial.read();
    gps.encode(c);
    bytesRead++;
    currentLine += c;
    if (c == '\n') {
      dataReceived += currentLine;
      currentLine = "";
    } 
  }

  // Set time only once when GPS date and time are valid
  if (!localTimeSet && gps.date.isValid() && gps.time.isValid() && gps.date.isUpdated() &&  gps.time.isUpdated() ) {
    time_t t = getGPSTime();
    if (t > 100000) {
      struct timeval now = { t, 0 };
      settimeofday(&now, nullptr);
      localTimeSet = true;  // Prevent further updates
    }
  }

  if (now - lastBroadcast > updateInterval) {
    lastBroadcast = now;
    sendGpsData();
  }

  if (now - lastScreenUpdate >= updateInterval) {
    //update screen
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("WIFI: ");
    if (WiFi.status() == WL_CONNECTED) {
      display.println(WiFi.localIP());
    }
    else {
      display.println("N/A");
    }

    if (gps.date.isUpdated() && gps.time.isUpdated()) {
      display.print(gps.date.year());
      display.print('/');
      if (gps.date.month() < 10) display.print('0');
      display.print(gps.date.month());
      display.print('/');
      if (gps.date.day() < 10) display.print('0');
      display.print(gps.date.day());
      display.print(' ');
      // Display time (UTC)
      if (gps.time.hour() < 10) display.print('0');
      display.print(gps.time.hour());
      display.print(':');
      if (gps.time.minute() < 10) display.print('0');
      display.print(gps.time.minute());
      display.print(':');
      if (gps.time.second() < 10) display.print('0');
      display.println(gps.time.second());
    }
    else {
      display.println("----/--/-- --:--:--");
    }
    display.println("[" + strNow + "] Delay=" + String(now - lastScreenUpdate));
    display.println("[" + strNow + "] Data=" + String(bytesRead));

    if (gps.location.isValid()) {
      String stale = gps.location.isUpdated() ? "" : "*";
      String sat = String(gps.satellites.value()) + stale;
      String lat = String(gps.location.lat(), 6) + stale;
      String lng = String(gps.location.lng(), 6) + stale;
      display.println("Sat: " + sat);
      display.println("Lat: " + lat);
      display.println("Lng: " + lng);
    }
    else {
      display.println("Sat: N/A");
      display.println("Lat: N/A");
      display.println("Lng: N/A");
    }
    display.display();

    //csv 
    String csvData = "";
    if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
        // Format: YYYY/MM/DD HH:MM:SS
        char timestamp[25];
        snprintf(timestamp, sizeof(timestamp), "%04d/%02d/%02d %02d:%02d:%02d",
                gps.date.year(),
                gps.date.month(),
                gps.date.day(),
                gps.time.hour(),
                gps.time.minute(),
                gps.time.second());
      String lat = String(gps.location.lat(), 6);
      String lng = String(gps.location.lng(), 6);
      csvData = String(timestamp) + "," + lat + "," + lng;
    }

    if (telnetClient && telnetClient.connected()) {
      telnetClient.print(dataReceived);

      telnetClient.println();
      telnetClient.println();
      telnetClient.println("fileName: " + lastFileName);
      if (localTimeSet) {
        time_t gpsNow = getGPSTime();
        time_t localNow = time(nullptr);
        telnetClient.print("localNow: ");
        telnetClient.println(ctime(&localNow));
        telnetClient.print("gpsNow: ");
        telnetClient.println(ctime(&gpsNow));
        telnetClient.println("csvData: " + (csvData == "" ? "N/A" : csvData));
        telnetClient.println("time diff: " + String(abs(localNow - gpsNow)));
        // if (abs(localNow - gpsNow) > 2) { // Only update if >2 sec drift
        //   struct timeval tNow = { gpsNow, 0 };
        //   settimeofday(&tNow, nullptr);
        // }
      }

      telnetClient.println();
      telnetClient.println();
    }
    //write to file
    if (localTimeSet && sdAvailable) {
      String fileName = getTimestampForFileName();
      String rawGpsFileName = "raw/" + getFolderPath() + "/gps_" + fileName + ".log";
      String parsedCsvFileName = "parsed/" + getFolderPath() + "/gps_" + fileName + ".csv";
      if (fileName != lastFileName) {
        if (gpsFile) gpsFile.close();
        if (csvFile) csvFile.close();
        gpsFile = SD.open(rawGpsFileName.c_str(), FILE_WRITE);
        csvFile = SD.open(parsedCsvFileName.c_str(), FILE_WRITE);
        lastFileName = fileName;
      }
      gpsFile.print(dataReceived);
      gpsFile.flush();
      if (csvData != "") {
        csvFile.println(csvData);
        csvFile.flush();
      }
    }

    bytesRead = 0;
    dataReceived = "";
    lastScreenUpdate = now;
  }
}
