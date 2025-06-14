#include <WiFi.h>
#include <esp32cam.h>
#include <WebServer.h>

const char* WIFI_SSID = "Redmi 13";
const char* WIFI_PASS = "99999999";

WebServer server(80);

static auto streamRes = esp32cam::Resolution::find(320, 240); // resolution for smooth streaming

void handleStream() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n"
                    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  client.print(response);

  while (client.connected()) {
    auto frame = esp32cam::capture();
    if (frame == nullptr) {
      Serial.println("Capture failed");
      break;
    }

    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", frame->size());
    frame->writeTo(client);
    client.print("\r\n");

    delay(50); 
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(streamRes);
    cfg.setBufferCount(1);
    cfg.setJpeg(70); // lower quality = faster

    bool ok = Camera.begin(cfg);
    Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
    if (!ok) {
      while (true) delay(100);
    }
  }

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Stream URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream");

  server.on("/stream", handleStream);
  server.begin();
}

void loop() {
  server.handleClient();
}