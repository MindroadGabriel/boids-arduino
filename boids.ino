#include <Vector.h>

#include <Vector_datatype.h>
#include <quaternion_type.h>
#include <vector_type.h>

#include <Bounce2.h>

#include <Adafruit_SSD1306.h>
#include <splash.h>

#include "FastIMU.h"
#include <Wire.h>

#define ARDUINO_ON_NEW_BOARD 0
#define IMU_ADDRESS 0x69    //Change to the address of the IMU
//#define PERFORM_CALIBRATION //Comment to disable startup calibration
BMI160 IMU;               //Change to the name of any supported IMU! 

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data

Bounce button1 = Bounce();
Bounce button2 = Bounce();

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define CIRCLE_RADIUS 3
#define BOIDS_SCALE 1
#define TIME_SCALE (3.0f)
const int NUM_MAX_BOIDS = 10;
const float sep_distance = 15.0f;
const float ali_distance = 25;
const float coh_distance = 30;
const float pred_distance = 15;
const float sep_weight = 1.5;
const float ali_weight = 0.8;
const float coh_weight = 1.2;
const float pred_weight = 2.0;
const float r = 2.0f;
const float maxforce = 0.03f * TIME_SCALE; // Maximum steering force
const float pred_maxforce = 0.1 * TIME_SCALE;
const float maxspeed = 2.0; // Maximum speed

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pq  §in # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

float distance(vec3_t from, vec3_t to);
void limit(vec3_t &vec, float max_magnitude);

class Boid {
public:
    vec3_t position;
    vec3_t velocity;
    Boid() {}
    void initialize(float x, float y) {
        randomize_direction();
        position = vec3_t(x, y);
    }
    void randomize_direction() {
        float angle = random(0.0f, 2.0f*PI);
        velocity = vec3_t(cos(angle), sin(angle));
    }
    void run(Vector<Boid> &boids, vec3_t predator_location, bool log) {
        vec3_t acceleration = flock(boids, predator_location, log);
        update(acceleration, log);
        borders();
        render();
    }
    vec3_t flock(Vector<Boid> &boids, vec3_t predator_location, bool log) {
        vec3_t sep = separate(boids);
        vec3_t ali = align(boids);
        vec3_t coh = cohesion(boids);
        vec3_t pred = predator(predator_location);
        sep *= sep_weight;
        ali *= ali_weight;
        coh *= coh_weight;
        pred *= pred_weight;
        if (log) {
            Serial.print("flock: ");
            Serial.print(sep.x);
            Serial.print(":");
            Serial.print(sep.y);
            Serial.print(", ");
            Serial.print(ali.x);
            Serial.print(":");
            Serial.print(ali.y);
            Serial.print(", ");
            Serial.print(coh.x);
            Serial.print(":");
            Serial.print(coh.y);
            Serial.print("  ");
        }

        vec3_t acceleration = vec3_t(0, 0);
        acceleration += sep;
        acceleration += ali;
        acceleration += coh;
        acceleration += pred;
        return acceleration;
    }
    void update(vec3_t acceleration, bool log) {
        if (log) {
            Serial.print("update: ");
            Serial.print(acceleration.x);
            Serial.print(":");
            Serial.print(acceleration.y);
            Serial.print(", ");
            Serial.print(velocity.x);
            Serial.print(":");
            Serial.print(velocity.y);
            Serial.print(", ");
            Serial.print(position.x);
            Serial.print(":");
            Serial.print(position.y);
            Serial.print("  ");
        }
        // Update velocity
        velocity += acceleration;
        // Limit speed
        limit(velocity, maxspeed);
        position += velocity * TIME_SCALE;
    }

    vec3_t seek(vec3_t target) {
        // A vector pointing from the position to the target
        vec3_t desired = target - position;
        // Scale to maximum speed
        desired.norm();
        desired *= maxspeed;

        vec3_t steer = desired - velocity;
        limit(steer, maxforce);
        return steer;
    }

    void render() {
        int16_t top_left_x = position.x * BOIDS_SCALE;
        int16_t top_left_y = position.y * BOIDS_SCALE;
        display.drawPixel(top_left_x, top_left_y, WHITE);
        display.drawPixel(top_left_x + 1, top_left_y, WHITE);
        display.drawPixel(top_left_x, top_left_y + 1, WHITE);
        display.drawPixel(top_left_x + 1, top_left_y + 1, WHITE);
    }

    void borders() {
        float width = (float)SCREEN_WIDTH / BOIDS_SCALE;
        float height = (float)SCREEN_HEIGHT / BOIDS_SCALE;
        if (position.x < -r) position.x = width + r;
        if (position.y < -r) position.y = height + r;
        if (position.x > width + r) position.x = -r;
        if (position.y > height + r) position.y = -r;
    }

    // Separation
    // Method checks for nearby boids and steers away
    vec3_t separate(Vector<Boid> &boids) {
        vec3_t steer = vec3_t(0, 0, 0);
        int count = 0;
        for (Boid &other : boids) {
            float d = distance(position, other.position);
            if ((d > 0) && (d < sep_distance)) {
                vec3_t diff = position - other.position;
                diff.norm();
                diff /= d;
                steer += diff;
                count++;
            }
        }
        if (count > 0) {
            steer /= (float)count;
        }

        if (steer.mag() > 0) {
            // Implement Reynolds: Steering = Desired - Velocity
            steer.norm();
            steer *= maxspeed;
            steer -= velocity;
            limit(steer, maxforce);
        }
        return steer;
    }

    // Alignment
    // For every nearby boid in the system, calculate the average velocity
    vec3_t align(Vector<Boid> &boids) {
        vec3_t sum = vec3_t(0, 0);
        int count = 0;
        for (Boid &other : boids) {
            float d = distance(position, other.position);
            if ((d > 0) && (d < ali_distance)) {
                sum += other.velocity;
                count++;
            }
        }
        if (count > 0) {
            sum /= (float)count;
            sum.norm();
            sum *= maxspeed;
            vec3_t steer = sum - velocity;
            limit(steer, maxforce);
            return steer;
        } else {
            return vec3_t(0, 0);
        }
    }

    // Cohesion
    // For the average position (i.e. center) of all nearby boids,
    // calculate steering vector towards that position
    vec3_t cohesion(Vector<Boid> &boids) {
        vec3_t sum = vec3_t(0, 0);
        int count = 0;
        for (Boid other : boids) {
            float d = distance(position, other.position);
            if ((d > 0) && (d < coh_distance)) {
                sum += other.position;
                count++;
            }
        }
        if (count > 0) {
            sum /= count;
            return seek(sum);
        } else {
            return vec3_t(0, 0);
        }
    }
    // Predator
    // Method avoids the predator chasing boids
    vec3_t predator(vec3_t predator_location) {
        vec3_t steer = vec3_t(0, 0, 0);
        float d = distance(position, predator_location);
        if ((d > 0) && (d < pred_distance)) {
            vec3_t diff = position - predator_location;
            diff.norm();
            diff /= d;
            steer += diff;
        }

        if (steer.mag() > 0) {
            // Implement Reynolds: Steering = Desired - Velocity
            steer.norm();
            steer *= maxspeed;
            steer -= velocity;
            limit(steer, pred_maxforce);
        }
        return steer;
    }

float distance(vec3_t from, vec3_t to) {
    return (from - to).mag();
}
void limit(vec3_t &vec, float max_magnitude) {
    float mag = vec.mag();
    if (mag > 0 && mag > max_magnitude) {
        vec /= mag;
        vec *= max_magnitude;
    }
}

void writeString(char* text) {
  char* c = text;
  while(*c) {
    display.write(*c);
    c++;
  }
}


void writeFloatString(float n) {
    char buffer[128];
#if BOARD_VENDORID == 0x2e8a && BOARD_PRODUCTID == 0x00c0
    // If we're on raspberry pi pico
    sprintf(buffer, "%.2f", n);
#else
    dtostrf(n, 4, 2, buffer);
#endif
    int len = strlen(buffer);
    int desired_length = 6;
    for (int i = 0; i < len - desired_length; ++i) {
        writeString(" ");
    }
    writeString(buffer);
}
void setup() {
  Wire.begin();
  Wire.setClock(10000); //400khz clock
  Serial.begin(9600);
  delay(1000);

#if BOARD_VENDORID == 0x2e8a && BOARD_PRODUCTID == 0x00c0
    // If we're on raspberry pi pico
    unsigned button1_pin = 7;
    unsigned button2_pin = 8;
#else
    unsigned button1_pin = 5;
    unsigned button2_pin = 4;
#endif
    pinMode(button1_pin,INPUT_PULLUP);
    button1.attach(button1_pin);
    button1.interval(10);// interval in ms
    pinMode(button2_pin,INPUT_PULLUP);
    button2.attach(button2_pin);
    button2.interval(10); // interval in ms

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }// */
  display.clearDisplay();
  display.display();
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.display();

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
}

void loop() {
    Boid storage_array[NUM_MAX_BOIDS];
    Vector<Boid> boids(storage_array);
    for (int i = 0; i < NUM_MAX_BOIDS; ++i) {
        Boid boid = Boid();
        boid.initialize(10, 10);
        boids.push_back(boid);
    }
    while (true) {
        button1.update();
        button2.update();
        if (button1.fell()) {
            display.clearDisplay();
            display.setCursor(0, 0);
            writeString("Calibrating.\nKeep level.");
            display.display();
            delay(1000);

            IMU.calibrateAccelGyro(&calib);
            IMU.init(calib, IMU_ADDRESS);

            display.clearDisplay();
            display.setCursor(0, 0);
            writeString("Calibration done!");
            display.display();
            delay(800);
        }

        if (button2.fell()) {
            for (Boid &boid : boids) {
                boid.randomize_direction();
            }
        }

        IMU.update();
        IMU.getAccel(&accelData);
        display.clearDisplay();
        display.setCursor(0, 0);

        float factor = 2.5;
#if (BOARD_VENDORID == 0x2e8a && BOARD_PRODUCTID == 0x00c0) || ARDUINO_ON_NEW_BOARD != 0
        float rawX = -accelData.accelX;
        float rawY = accelData.accelY;
#else
        float rawX = accelData.accelY;
        float rawY = accelData.accelX;
#endif
        Serial.print("update: ");
        Serial.print(rawX);
        Serial.print(", ");
        Serial.print(rawY);
        Serial.print(", ");
        Serial.print(accelData.accelZ);
        Serial.print("\n");

        int16_t x = CIRCLE_RADIUS + ((rawX*factor + 1) / 2) * (SCREEN_WIDTH - CIRCLE_RADIUS);
        int16_t y = CIRCLE_RADIUS + ((rawY*factor + 1) / 2) * (SCREEN_HEIGHT - CIRCLE_RADIUS);
        vec3_t predator_location = vec3_t(x, y);
        display.drawCircle(x, y, CIRCLE_RADIUS, SSD1306_WHITE);
        bool first = false;
        for (Boid &boid : boids) {
            boid.run(boids, predator_location, first);
            first = false;
        }
        Serial.println();
        display.display();
    }
}