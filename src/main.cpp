#include <M5Unified.h>
#include <M5Unit-IMU-Pro-Mini.h>
#include <Adafruit_BMP280.h>
#include <vector>

std::vector<float> magX, magY, magZ;


#define BIM270_SENSOR_ADDR 0x68
#define BMP280_SENSOR_ADDR 0x76

BMI270::BMI270 bmi270;
Adafruit_BMP280 bmp(&Wire);

void setup() {
    M5.begin();
    M5.Ex_I2C.begin();

    unsigned status = bmp.begin(BMP280_SENSOR_ADDR);
    if (!status) {
        Serial.println(
            F("Could not find a valid BMP280 sensor, check wiring or "
              "try a different address!"));
        Serial.print("SensorID was: 0x");
        Serial.println(bmp.sensorID(), 16);
        while (1) delay(10);
    }

    bmi270.init(I2C_NUM_0, BIM270_SENSOR_ADDR);
}

void loop(void) {
    // put your main code here, to run repeatedly:
    float x, y, z;

    if (bmi270.accelerationAvailable()) {
        bmi270.readAcceleration(x, y, z);

        Serial.print("accel: \t");
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.print(z);
        Serial.println();
    }

    if (bmi270.gyroscopeAvailable()) {
        bmi270.readGyroscope(x, y, z);

        Serial.print("gyro: \t");
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.print(z);
        Serial.println();
    }

    if (bmi270.magneticFieldAvailable()) {
        int16_t mx, my, mz = 0;
        bmi270.readMagneticField(mx, my, mz);

        Serial.print("mag: \t");
        Serial.print(mx);
        Serial.print('\t');
        Serial.print(my);
        Serial.print('\t');
        Serial.print(mz);
        Serial.println();
    }

    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();
    delay(500);
}
/*
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Imu.begin();
  M5.Lcd.setTextSize(2);
  M5.Lcd.println("Collecting... Move M5 in all directions.");
}

void loop() {
  float x, y, z;
  if (M5.Imu.getMagnet(&x, &y, &z)) {
    magX.push_back(x);
    magY.push_back(y);
    magZ.push_back(z);

    M5.Lcd.setCursor(0, 30);
    M5.Lcd.printf("X:%.1f Y:%.1f Z:%.1f ", x, y, z);
    delay(100);
  }

  M5.update();
  if (M5.BtnA.wasPressed()) {
    const int N = magX.size();
    if (N < 10) {
      M5.Lcd.println("Not enough data.");
      return;
    }

    // X = [x y z 1], y = x^2 + y^2 + z^2
    float Sxx = 0, Sxy = 0, Sxz = 0, Sx1 = 0;
    float Syy = 0, Syz = 0, Sy1 = 0;
    float Szz = 0, Sz1 = 0;
    float S11 = N;

    float Sy = 0, Sx_y = 0, Sy_y = 0, Sz_y = 0, S1_y = 0;

    for (int i = 0; i < N; ++i) {
      float x = magX[i], y = magY[i], z = magZ[i];
      float r2 = x * x + y * y + z * z;

      Sxx += x * x;
      Sxy += x * y;
      Sxz += x * z;
      Sx1 += x;

      Syy += y * y;
      Syz += y * z;
      Sy1 += y;

      Szz += z * z;
      Sz1 += z;

      Sy += r2;
      Sx_y += x * r2;
      Sy_y += y * r2;
      Sz_y += z * r2;
      S1_y += r2;
    }

    // Solve [A]{b} = {Y} using normal equations (no matrix lib)
    // A: 4x4 matrix, b: [Bx By Bz D]T
    float A[4][4] = {
      {Sxx, Sxy, Sxz, Sx1},
      {Sxy, Syy, Syz, Sy1},
      {Sxz, Syz, Szz, Sz1},
      {Sx1, Sy1, Sz1, S11}
    };

    float B[4] = {Sx_y, Sy_y, Sz_y, S1_y};

    // Gaussian elimination (naive)
    for (int i = 0; i < 4; ++i) {
      // Normalize row
      float diag = A[i][i];
      for (int j = 0; j < 4; ++j) A[i][j] /= diag;
      B[i] /= diag;
      // Eliminate below
      for (int k = i + 1; k < 4; ++k) {
        float f = A[k][i];
        for (int j = 0; j < 4; ++j) A[k][j] -= f * A[i][j];
        B[k] -= f * B[i];
      }
    }

    // Back substitution
    float params[4];
    for (int i = 3; i >= 0; --i) {
      params[i] = B[i];
      for (int j = i + 1; j < 4; ++j) {
        params[i] -= A[i][j] * params[j];
      }
    }

    float bx = params[0] / 2.0f;
    float by = params[1] / 2.0f;
    float bz = params[2] / 2.0f;
    float d = params[3];

    // Compute average radius (for scale normalization)
    float radius_sum = 0;
    for (int i = 0; i < N; ++i) {
      float dx = magX[i] - bx;
      float dy = magY[i] - by;
      float dz = magZ[i] - bz;
      radius_sum += sqrt(dx * dx + dy * dy + dz * dz);
    }
    float avg_radius = radius_sum / N;

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Normal Equation Fit:");
    M5.Lcd.printf("Offset = { %.3f, %.3f, %.3f }\n", bx, by, bz);
    M5.Lcd.printf("Avg radius = %.3f\n", avg_radius);

    // そのまま使用可能なスケール係数（1で正規化する場合）
    M5.Lcd.printf("Use scale = 1.0 / radius\n");

    while (true) delay(100);
  }
}
*/