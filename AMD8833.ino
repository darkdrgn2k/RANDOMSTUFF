

/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

// Load Wi-Fi library
#include <WiFi.h>

#include <Wire.h>
#include <Adafruit_AMG88xx.h>

// Replace with your network credentials
const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";


Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output26State = "off";
String output27State = "off";

// Assign output variables to GPIO pins
const int output26 = 26;
const int output27 = 27;
#define MAX_COLOR 128

const int color[]= {
0x08100d,0x0a1512,0x0d1a16,0x0f1f1a,0x12231f,0x142824,0x172d28,
0x19322c,0x1c3730,0x1e3c35,0x214139,0x23463d,0x254b41,0x285046,
0x2a554a,0x2d5a4e,0x2f5f52,0x326457,0x34695b,0x376d60,0x397264,
0x3c7769,0x3e7c6d,0x418171,0x438675,0x448b70,0x448f65,0x44935a,
0x44984e,0x449c44,0x44a039,0x44a52d,0x44a922,0x44ad18,0x44b10d,
0x44b601,0x44b9f6,0x44bdeb,0x44c2e0,0x44c6d5,0x44caca,0x44cfbe,
0x44d3b4,0x44d7a9,0x44dc9d,0x44e092,0x44e487,0x44e97c,0x44ed71,
0x44f166,0x44f65a,0x44fa4f,0x44fe45,0x4bfd43,0x51fd41,0x57fc3f,
0x5dfc3d,0x63fb3c,0x6afb39,0x70fa38,0x76fa35,0x7cf934,0x83f931,
0x89f830,0x8ff72e,0x95f72c,0x9cf62a,0xa2f628,0xa8f526,0xaef524,
0xb5f423,0xbbf420,0xc1f31f,0xc7f31c,0xcef21b,0xd4f119,0xdaf117,
0xe0f015,0xe6f013,0xedef11,0xf0ee12,0xf0eb18,0xf0e81d,0xf1e622,
0xf1e327,0xf1e02c,0xf1dd32,0xf1db36,0xf1d83c,0xf1d541,0xf2d346,
0xf2d04b,0xf2cd51,0xf2ca56,0xf2c85b,0xf2c560,0xf2c266,0xf3bf6b,
0xf3bd70,0xf3ba75,0xf3b77a,0xf3b57f,0xf3b284,0xf3af8a,0xf4ac8f,
0xf4aa94,0xf4a799,0xf4a49f,0xf4a1a2,0xf59da6,0xf599aa,0xf696ac,
0xf692b0,0xf68eb4,0xf78bb7,0xf787bb,0xf883bf,0xf880c2,0xf87cc5,
0xf978c9,0xf975cc,0xfa71d0,0xfa6dd4,0xfa6ad7,0xfb66db,0xfb62de,
0xfc5fe1,0xfc5be5};


//INTEROPOLATE

#define AMG_ROWS 8
#define AMG_COLS 8

#define INTERPOLATED_COLS 24
#define INTERPOLATED_ROWS 24


float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
float cubicInterpolate(float p[], float x);
float bicubicInterpolate(float p[], float x, float y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, 
                       float *dest, uint8_t dest_rows, uint8_t dest_cols);


float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y) {
  if (x < 0)        x = 0;
  if (y < 0)        y = 0;
  if (x >= cols)    x = cols - 1;
  if (y >= rows)    y = rows - 1;
  return p[y * cols + x];
}

void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f) {
  if ((x < 0) || (x >= cols)) return;
  if ((y < 0) || (y >= rows)) return;
  p[y * cols + x] = f;
}

// src is a grid src_rows * src_cols
// dest is a pre-allocated grid, dest_rows*dest_cols
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, 
                       float *dest, uint8_t dest_rows, uint8_t dest_cols) {
  float mu_x = (src_cols - 1.0) / (dest_cols - 1.0);
  float mu_y = (src_rows - 1.0) / (dest_rows - 1.0);

  float adj_2d[16]; // matrix for storing adjacents
  
  for (uint8_t y_idx=0; y_idx < dest_rows; y_idx++) {
    for (uint8_t x_idx=0; x_idx < dest_cols; x_idx++) {
       float x = x_idx * mu_x;
       float y = y_idx * mu_y;
       //Serial.print("("); Serial.print(y_idx); Serial.print(", "); Serial.print(x_idx); Serial.print(") = ");
       //Serial.print("("); Serial.print(y); Serial.print(", "); Serial.print(x); Serial.print(") = ");
       get_adjacents_2d(src, adj_2d, src_rows, src_cols, x, y);
       /*
       Serial.print("[");
       for (uint8_t i=0; i<16; i++) {
         Serial.print(adj_2d[i]); Serial.print(", ");
       }
       Serial.println("]");
       */
       float frac_x = x - (int)x; // we only need the ~delta~ between the points
       float frac_y = y - (int)y; // we only need the ~delta~ between the points
       float out = bicubicInterpolate(adj_2d, frac_x, frac_y);
       //Serial.print("\tInterp: "); Serial.println(out);
       set_point(dest, dest_rows, dest_cols, x_idx, y_idx, out);
    }
  }
}

// p is a list of 4 points, 2 to the left, 2 to the right
float cubicInterpolate(float p[], float x) {
    float r = p[1] + (0.5 * x * (p[2] - p[0] + x*(2.0*p[0] - 5.0*p[1] + 4.0*p[2] - p[3] + x*(3.0*(p[1] - p[2]) + p[3] - p[0]))));
  /*
    Serial.print("interpolating: ["); 
    Serial.print(p[0],2); Serial.print(", ");
    Serial.print(p[1],2); Serial.print(", ");
    Serial.print(p[2],2); Serial.print(", ");
    Serial.print(p[3],2); Serial.print("] w/"); Serial.print(x); Serial.print(" = ");
    Serial.println(r);
  */
    return r;
}

// p is a 16-point 4x4 array of the 2 rows & columns left/right/above/below
float bicubicInterpolate(float p[], float x, float y) {
    float arr[4] = {0,0,0,0};
    arr[0] = cubicInterpolate(p+0, x);
    arr[1] = cubicInterpolate(p+4, x);
    arr[2] = cubicInterpolate(p+8, x);
    arr[3] = cubicInterpolate(p+12, x);
    return cubicInterpolate(arr, y);
}

// src is rows*cols and dest is a 4-point array passed in already allocated!
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y) {
    //Serial.print("("); Serial.print(x); Serial.print(", "); Serial.print(y); Serial.println(")");
    // pick two items to the left
    dest[0] = get_point(src, rows, cols, x-1, y);
    dest[1] = get_point(src, rows, cols, x, y);
    // pick two items to the right
    dest[2] = get_point(src, rows, cols, x+1, y);
    dest[3] = get_point(src, rows, cols, x+2, y);
}


// src is rows*cols and dest is a 16-point array passed in already allocated!
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y) {
    //Serial.print("("); Serial.print(x); Serial.print(", "); Serial.print(y); Serial.println(")");
    float arr[4];
    for (int8_t delta_y = -1; delta_y < 3; delta_y++) { // -1, 0, 1, 2
        float *row = dest + 4 * (delta_y+1); // index into each chunk of 4
        for (int8_t delta_x = -1; delta_x < 3; delta_x++) { // -1, 0, 1, 2
            row[delta_x+1] = get_point(src, rows, cols, x+delta_x, y+delta_y);
        }
    }
}
//END

void setup() {
  Serial.begin(115200);
  // Initialize the output variables as outputs
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  server.begin();


      bool status;
    
    // default settings
    status = amg.begin();
    if (!status) {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
        while (1);
    }
    
    Serial.println("-- Pixels Test --");

    Serial.println();

    delay(100); // let sensor boot up
}
int high=0;



void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
                  
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>* {  margin: 0px auto; padding:0; text-align: center; font-size:3px; }");
            client.println(".a {display:inline-block; width:10px; height:10px; font-size:5px; overflow:hidden;} ");
            client.println("</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");


              amg.readPixels(pixels);

                            

     int32_t t = millis();
     
float pixelsLarge[INTERPOLATED_ROWS * INTERPOLATED_COLS];

  interpolate_image(pixels, AMG_ROWS, AMG_COLS, pixelsLarge, INTERPOLATED_ROWS, INTERPOLATED_COLS);
Serial.print("Interpolation took "); Serial.print(millis()-t); Serial.println(" ms");



    for(int i=1; i<=INTERPOLATED_ROWS * INTERPOLATED_COLS; i++){

    
      int index=(pixelsLarge[i-1])/100*MAX_COLOR;      

      
      if (index>=MAX_COLOR)index=MAX_COLOR-1;
      int cid=color[index];
      
      client.print("<div class='a' style='background: #");
      client.print(String(cid, HEX));
      client.print(";'>");
      //client.print(pixelsLarge[i-1]);
      client.print("</div>");
      if( i%24 == 0 ) client.print("<div style='display:block; overflow:hidden; width:1px; height:1px;'></div>");
    }



    
    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){

      Serial.print(pixels[i-1]);
      Serial.print(" -- Index -- ");
      int index=(pixels[i-1]+18)/110*MAX_COLOR;      
      Serial.print(index);
      Serial.print("--color--");
      Serial.println(color[index]);
      
      if (index>=MAX_COLOR)index=MAX_COLOR-1;
      int cid=color[index];
      Serial.println(cid);      

      
      client.print("<div style='display:inline-block; width:30px; height:30px; font-size:6px; background: #");
      client.print(String(cid, HEX));
      Serial.println(String(cid, HEX));
      client.print(";'>");
      client.print(pixels[i-1]);
      client.print("</div>");
      if( i%8 == 0 ) client.print("<br />");
    }
    
    client.println();
    
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
