#include <pgmspace.h>

#define SECRET
#define THINGNAME ""

const char WIFI_SSID[] = "<your ssid>";
const char WIFI_PASSWORD[] = "passphrase";
const char MQTT_ENDPOINT[] = "192.168.10.1"; // Broker IP

// Root CA 1
static const char MQTT_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----

rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

// Device Certificate
static const char MQTT_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)KEY";

// Device Private Key
static const char MQTT_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----

-----END RSA PRIVATE KEY-----
)KEY";