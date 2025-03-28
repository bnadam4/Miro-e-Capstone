#include <WiFi.h>
#include <WebServer.h>

// Replace with your network credentials
const char *ssid = "MiroCap";
const char *password = "password_placeholder";

// Initialize the WebServer on port 80
WebServer server(80);

// Variables to store the received UART data
uint8_t data1 = 0;
uint8_t data2 = 0;
uint8_t data3 = 0;
uint8_t data4 = 0;
uint8_t data5 = 0;

// Static IP configuration
IPAddress local_ip(192, 168, 1, 103);  // Set your desired static IP address
IPAddress gateway(192, 168, 1, 1);     // Set your gateway (usually your router's IP)
IPAddress subnet(255, 255, 255, 0);    // Set your subnet mask

#define TX_PIN 43
#define RX_PIN 44

void setup() {
  // Start Serial communication for debugging
  Serial.begin(115200);
  Serial.println("Starting program.");
  delay(1000); // Wait a bit for the Serial Monitor to initialize

  Serial.println("Starting ESP32...");
  
  // Set the static IP configuration
  WiFi.config(local_ip, gateway, subnet);

  // Connect to Wi-Fi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("Connected to WiFi");

  // Print the IP address
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize Serial1 (UART1)
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Define request handlers
  server.on("/", handleRoot);
  server.on("/getdata", handleGetData); // Endpoint to fetch the 5 bytes
  server.on("/getbehavior", handleGetBehavior); // Endpoint to fetch behavior description
  server.on("/senddata", HTTP_POST, handleSendData); // Endpoint to send data to the UART

  // Start the web server
  server.begin();
  Serial.println("Web server started.");
}

void loop() {
  // Handle client requests
  server.handleClient();

  // Non-blocking UART data reading
  readUARTData();
}

// Non-blocking function to read UART data
void readUARTData() {
  if (Serial1.available() >= 5) { // Wait until at least 5 bytes are available
    // Read the incoming bytes
    data1 = Serial1.read(); // First byte
    data2 = Serial1.read(); // Second byte
    data3 = Serial1.read(); // Third byte
    data4 = Serial1.read(); // Fourth byte
    data5 = Serial1.read(); // Fifth byte

    // Print the received data for debugging
    Serial.println("\nReceived numbers: ");
    Serial.print(data1); Serial.print(" ");
    Serial.print(data2); Serial.print(" ");
    Serial.print(data3); Serial.print(" ");
    Serial.print(data4); Serial.print(" ");
    Serial.println(data5);

    // Process the received data
    String behaviorDescription = processBehavior(data1, data2, data3, data4, data5);
    Serial.println(behaviorDescription);
  }
}

// Handle /getdata URL to send the 5 bytes to the client
void handleGetData() {
  // Create a JSON response with the 5 bytes
  String response = "{\"data1\":" + String(data1) + ",";
  response += "\"data2\":" + String(data2) + ",";
  response += "\"data3\":" + String(data3) + ",";
  response += "\"data4\":" + String(data4) + ",";
  response += "\"data5\":" + String(data5) + "}";

  // Send the response as JSON
  server.send(200, "application/json", response);
}

// Handle /getbehavior URL to return the behavior description
void handleGetBehavior() {
  if (server.hasArg("data1") && server.hasArg("data2") && server.hasArg("data3") && server.hasArg("data4") && server.hasArg("data5")) {
    // Parse the received data
    uint8_t data1 = server.arg("data1").toInt();
    uint8_t data2 = server.arg("data2").toInt();
    uint8_t data3 = server.arg("data3").toInt();
    uint8_t data4 = server.arg("data4").toInt();
    uint8_t data5 = server.arg("data5").toInt();

    // Get the behavior description
    String behaviorDescription = processBehavior(data1, data2, data3, data4, data5);

    // Send the behavior description as plain text
    server.send(200, "text/plain", behaviorDescription);
  } else {
    // Send an error response if parameters are missing
    server.send(400, "text/plain", "Missing parameters. Expected data1, data2, data3, data4, data5.");
  }
}

// Handle /senddata URL to receive 5 bytes from the client and send it over UART
void handleSendData() {
  // Read the POST request body to extract the 5 bytes
  if (server.hasArg("plain")) {
    String data = server.arg("plain");
    
    // Assuming the data is in the form of "1,2,0,0,1"
    int byteValues[5];
    int i = 0;
    char* token = strtok((char*)data.c_str(), ",");
    
    while (token != NULL && i < 5) {
      byteValues[i++] = atoi(token);
      token = strtok(NULL, ",");
    }
    
    if (i == 5) {
      // Send each byte over UART
      Serial1.write(byteValues[0]);
      Serial1.write(byteValues[1]);
      Serial1.write(byteValues[2]);
      Serial1.write(byteValues[3]);
      Serial1.write(byteValues[4]);

      Serial.print(byteValues[0]);
      Serial.print(byteValues[1]);
      Serial.print(byteValues[2]);
      Serial.print(byteValues[3]);
      Serial.print(byteValues[4]);
      
      // Respond to the client
      server.send(200, "application/json", "{\"status\":\"success\"}");
      Serial.println("Sent data over UART.");
    } else {
      // Invalid data
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid data format\"}");
    }
  } else {
    // No data in the request
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
  }
}

// Function to process and return behavior as a string
String processBehavior(uint8_t behavior, uint8_t voice, uint8_t unused, uint8_t data1, uint8_t data2) {
  String result = "";

  // Behavior selection
  switch (behavior) {
    case 1:
      result = "Interactive Standby";
      break;
    case 2:
      result = "Breathing Exercise";
      break;
    case 3:
      result = "Muscle Relaxation";
      break;
    case 4:
      result = "Audiobook";
      break;
    case 5:
      result = "Checklist";
      break;
    default:
      result = "Unknown Behavior";
      break;
  }

  // Voice selection
  String voiceSelection = (voice == 1) ? "Kevin" : (voice == 2) ? "Emily" : "Unknown";
  result += "<br>Voice: " + voiceSelection;

  // Additional details based on behavior
  switch (behavior) {
    case 2: // Breathing Exercise
      if (data1 > 0) {
        result += "<br>Cycle/Question Count: " + String(data1);
      }
      if (data2 == 2) {
        result += "<br>Action: Exit";
      } else if (data2 == 255) {
        result += "<br>Action: Breath Reset";
      } else {
        result += "<br>Action: Unknown";
      }
      break;
    case 3: // Muscle Relaxation
      if (data2 == 2) {
        result += "<br>Action: Exit";
      } else if (data2 == 3) {
        result += "<br>Action: Head Relaxation";
      } else if (data2 == 4) {
        result += "<br>Action: Tummy Relaxation";
      } else if (data2 == 5) {
        result += "<br>Action: Arms Relaxation";
      } else if (data2 == 6) {
        result += "<br>Action: Back Relaxation";
      } else {
        result += "<br>Action: Unknown";
      }
      break;
    case 4: // Audiobook
      if (data2 == 2) {
        result += "<br>Action: Exit";
      } else if (data2 == 3) {
        result += "<br>Story: Rumpelstiltskin";
      } else {
        result += "<br>Action: Unknown";
      }
      break;
    case 5: // Checklist
      if (data1 > 0) {
        result += "<br>Number of Items: " + String(data1);
      }
      if (data2 == 2) {
        result += "<br>Action: Exit";
      } else if (data2 == 3) {
        result += "<br>Time: Morning";
      } else if (data2 == 4) {
        result += "<br>Time: Afternoon";
      } else {
        result += "<br>Action: Unknown";
      }
      break;
    default:
      break;
  }

  return result;
}

// Handle root URL ("/") to display the 5 bytes in HTML format
void handleRoot() {
  // Create an HTML page with JavaScript for dynamic updates
  String html = R"=====(
<html>
<head>
  <title>ESP32 - UART Data Display</title>
  <script>
    function fetchData() {
      fetch('/getdata')
        .then(response => response.json())
        .then(data => {
          // Fetch the behavior description
          fetchBehaviorDescription(data.data1, data.data2, data.data3, data.data4, data.data5);
        })
        .catch(error => console.error('Error fetching data:', error));
    }

    function fetchBehaviorDescription(data1, data2, data3, data4, data5) {
      fetch('/getbehavior?data1=' + data1 + '&data2=' + data2 + '&data3=' + data3 + '&data4=' + data4 + '&data5=' + data5)
        .then(response => response.text())
        .then(description => {
          document.getElementById('behavior').innerHTML = description;
        })
        .catch(error => console.error('Error fetching behavior description:', error));
    }

    // Fetch data every 2 seconds
    setInterval(fetchData, 2000);
  </script>
</head>
<body>
  <h1>ESP32 - UART Data Display</h1>
  <p><strong>Behavior Description:</strong></p>
  <p id="behavior">No data received yet.</p>
</body>
</html>
)=====";

  // Send the HTML page as a response
  server.send(200, "text/html", html);
}
