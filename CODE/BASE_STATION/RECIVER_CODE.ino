#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <WiFiClientSecure.h>

// Define HC-12 pins
#define HC12_RX 16  // RX pin for HC-12 connected to ESP32 TX pin
#define HC12_TX 17  // TX pin for HC-12 connected to ESP32 RX pin

SoftwareSerial hc12Serial(HC12_RX, HC12_TX);

// WiFi settings
const char* ssid = "martin";
const char* password = "martin363";

// MQTT settings
const char* mqtt_server = "diplom-ny7qjv.a01.euc1.aws.hivemq.cloud";
const char* mqtt_username = "martin";
const char* mqtt_password = "Martin363";
const int mqtt_port = 8883;

WiFiClientSecure espClient;
PubSubClient client(espClient);

// MQTT topic
const char* mqtt_topic = "SKYSAFENET/flightdata/rec";

// Root CA for SSL connection
static const char* root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57
demyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

void callback(char* topic, byte* payload, unsigned int length) {
  // Handle incoming messages here
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(mqtt_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200); // Start the serial communication for debugging
  hc12Serial.begin(115200); // Start the HC-12 serial communication

  // WiFi setup
  Serial.println("\nConnecting to " + String(ssid));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected\nIP address: " + WiFi.localIP().toString());

  // MQTT setup
  espClient.setCACert(root_ca);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Check if data is available from HC-12
  if (hc12Serial.available()) {
    String receivedData = hc12Serial.readStringUntil('\n');
    Serial.println("Received Data: " + receivedData);

    // Split the received data
    float latitude = getValue(receivedData, ',', 1).toFloat();
    float longitude = getValue(receivedData, ',', 3).toFloat();
    float altitude = getValue(receivedData, ',', 5).toFloat();
    float velocity = getValue(receivedData, ',', 7).toFloat();
    float heading = getValue(receivedData, ',', 9).toFloat();
    int checksumReceived = getValue(receivedData, ',', 11).toInt();

    // Calculate checksum
    int checksumCalculated = latitude + longitude + altitude + velocity + heading;

    // Verify checksum
    if (checksumCalculated == checksumReceived) {
      Serial.println("Checksum verified successfully!");
      Serial.print("Latitude: ");
      Serial.println(latitude, 6);
      Serial.print("Longitude: ");
      Serial.println(longitude, 6);
      Serial.print("Altitude: ");
      Serial.println(altitude, 2);
      Serial.print("Velocity: ");
      Serial.println(velocity, 2);
      Serial.print("Heading: ");
      Serial.println(heading, 2);

      // Publish data to MQTT
      String payload = "Lat: " + String(latitude, 6) + ", Lon: " + String(longitude, 6) +
                       ", Alt: " + String(altitude, 2) + ", Vel: " + String(velocity, 2) +
                       ", Head: " + String(heading, 2);

      client.publish(mqtt_topic, payload.c_str());
      Serial.println("Published to MQTT: " + payload);
    } else {
      Serial.println("Checksum verification failed!");
    }
  }
  delay(500); // Delay to prevent flooding the serial monitor
}

// Function to split string based on delimiter and return specific part
String getValue(String data, char delimiter, int index) {
  int start = 0;
  int end = data.indexOf(delimiter);
  for (int i = 0; i < index; i++) {
    start = end + 1;
    end = data.indexOf(delimiter, start);
  }
  return data.substring(start, end);
}

