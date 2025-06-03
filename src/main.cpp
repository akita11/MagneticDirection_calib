#include <M5Unified.h>
#include <M5_IMU_PRO.h>
#include <Adafruit_BMP280.h>
#include <vector>
#include <EEPROM.h>

std::vector<int> magX, magY, magZ;

#define BIM270_SENSOR_ADDR 0x68
#define BMP280_SENSOR_ADDR 0x76

BMI270::BMI270 bmi270;
Adafruit_BMP280 bmp(&Wire);

uint16_t n = 0;
uint8_t fRunning = 0;
float x0, yy0, z0, a, b, c;

#define BG_COLOR_CALIB RED
#define BG_COLOR_WORK BLACK

const int EEPROM_SIZE = 32;
const int EEPROM_ADDR = 0;  // オフセット0から保
float calib[6];

void setup()
{
	auto cfg = M5.config();
	M5.begin(cfg);
	M5.Ex_I2C.begin();
	M5.Display.setTextSize(2);

	unsigned status = bmp.begin(BMP280_SENSOR_ADDR);
	if (!status)
	{
		printf("failed to find IMU Pro\n");
		while (1)
			;
	}
	bmi270.init(I2C_NUM_0, BIM270_SENSOR_ADDR);
	M5.update();
	if (M5.BtnA.isPressed())
	{
		fRunning = 0; // calibration mode
		M5.Display.fillScreen(BG_COLOR_CALIB);
	}
	else
	{
		fRunning = 3; // working mode
		M5.Display.fillScreen(BG_COLOR_WORK);
		x0 = -2.448;
		yy0 = -11.007;
		z0 = -34.202;
		a = 6.720 * 2;
		b = 8.866 * 2;
		c = 7.963 * 2;
		EEPROM.begin(EEPROM_SIZE);
		EEPROM.get(EEPROM_ADDR, calib);
		x0 = calib[0];
		yy0 = calib[1];
		z0 = calib[2];
		a = calib[3];
		b = calib[4];
		c = calib[5];
		printf("%.3f %.3f %.3f / %.3f %.3f %.3f\n", x0, yy0, z0, a, b, c);
	}
}

void loop(void)
{
	int16_t mx, my, mz = 0;
	if (bmi270.magneticFieldAvailable())
	{
		bmi270.readMagneticField(mx, my, mz);
		if (fRunning == 1)
		{
			magX.push_back(mx);
			magY.push_back(my);
			magZ.push_back(mz);
			M5.Display.fillRect(0, 0, 60, 50);
			M5.Display.setCursor(0, 0);
			M5.Display.printf("N=%d\n%d\n%d\n%d", n++, mx, my, mz);
			printf("%d %d %d\n", mx, my, mz);
			delay(10);
		}
		else if (fRunning == 3)
		{
			float fmx, fmy, fmz;
			fmx = ((float)mx - x0) / a;
			fmy = ((float)my - yy0) / b;
			fmz = ((float)mz - z0) / c;
			float norm = sqrt(fmx * fmx + fmy * fmy + fmz * fmz);
			fmx /= norm;
			fmy /= norm;
			fmz /= norm;
			float yaw = atan2(-fmy, fmx) * 180.0 / PI; // 方位（z軸回り）
			float pitch = asin(fmz) * 180.0 / PI;			 // 簡易的な仮定
			float roll = asin(fmy) * 180.0 / PI;			 // 簡易的な仮定
//			printf("%d %d %d : %.3f %.3f %3f : %.3f %.3f %.3f\n", mx, my, mz, fmx, fmy, fmz, yaw, pitch, roll);
			printf("Dir:,%d,%.3f,%.3f,%.3f\n", n++, yaw, pitch, roll);
			M5.Display.setCursor(0, 00);
			M5.Display.fillRect(0, 00, 240, 60, BG_COLOR_WORK);
			M5.Display.printf("Y %.1f\n", yaw);
			M5.Display.printf("P %.1f\n", pitch);
			M5.Display.printf("R %.1f\n", roll);
		}
		delay(20);
	}

	M5.update();
	if (M5.BtnA.wasClicked())
	{
		if (fRunning == 0)
		{
			M5.Display.fillScreen(BG_COLOR_CALIB);
			fRunning = 1;
			magX.clear();
			magY.clear();
			magZ.clear();
			printf("N=%d\n", magX.size());
			n = 0;
		}
		else if (fRunning == 1)
		{
			int N = magX.size();
			M5.Display.fillScreen(BG_COLOR_CALIB);
			M5.Display.setCursor(0, 0);
			M5.Display.printf("cap.N=%d\n", N);
			if (N < 100)
			{
				M5.Display.println("Not enough data.");
			}
			else
			{
				// 正規方程式に基づく最小二乗法
				// Solve Ax = b for [Bx By Bz D]^T
				float Sxx = 0, Sxy = 0, Sxz = 0, Sx1 = 0;
				float Syy = 0, Syz = 0, Sy1 = 0;
				float Szz = 0, Sz1 = 0;
				float S11 = N;

				float Sx_r2 = 0, Sy_r2 = 0, Sz_r2 = 0, S1_r2 = 0;

				for (int i = 0; i < N; ++i)
				{
					float x = magX[i], y = magY[i], z = magZ[i];
					printf("%d %d %.3f\n", i, magX[i], x);
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

					Sx_r2 += x * r2;
					Sy_r2 += y * r2;
					Sz_r2 += z * r2;
					S1_r2 += r2;
				}

				float A[4][4] = {
						{Sxx, Sxy, Sxz, Sx1},
						{Sxy, Syy, Syz, Sy1},
						{Sxz, Syz, Szz, Sz1},
						{Sx1, Sy1, Sz1, S11}};
				float B[4] = {Sx_r2, Sy_r2, Sz_r2, S1_r2};

				// ガウス消去
				for (int i = 0; i < 4; i++)
				{
					float pivot = A[i][i];
					for (int j = 0; j < 4; j++)
						A[i][j] /= pivot;
					B[i] /= pivot;

					for (int k = i + 1; k < 4; k++)
					{
						float factor = A[k][i];
						for (int j = 0; j < 4; j++)
							A[k][j] -= factor * A[i][j];
						B[k] -= factor * B[i];
					}
				}

				float X[4];
				for (int i = 3; i >= 0; i--)
				{
					X[i] = B[i];
					for (int j = i + 1; j < 4; j++)
					{
						X[i] -= A[i][j] * X[j];
					}
				}

				x0 = X[0] / 2.0;
				yy0 = X[1] / 2.0;
				z0 = X[2] / 2.0;

				// スケールは中心補正後の分散で近似
				float sumX2 = 0, sumY2 = 0, sumZ2 = 0;
				for (int i = 0; i < N; ++i)
				{
					float dx = magX[i] - x0;
					float dy = magY[i] - yy0;
					float dz = magZ[i] - z0;
					sumX2 += dx * dx;
					sumY2 += dy * dy;
					sumZ2 += dz * dz;
				}
				a = sqrt(sumX2 / N);
				b = sqrt(sumY2 / N);
				c = sqrt(sumZ2 / N);
				M5.Display.fillScreen(BLACK);
				M5.Display.setCursor(0, 0);
				M5.Display.printf("%.3f %.3f %.3f\n", x0, yy0, z0);
				M5.Display.printf("%.3f %.3f %.3f\n", a, b, c);
				// そのまま使用可能なスケール係数（1で正規化する場合）
				// M5.Display.printf("Use scale = 1.0 / radius\n");
				printf("%.3f\n%.3f\n%.3f\n", x0, yy0, z0);
				printf("%.3f\n%.3f\n%.3f\n", a, b, c);
				EEPROM.begin(EEPROM_SIZE);
				calib[0] = x0;
				calib[1] = yy0;
				calib[2] = z0;
				calib[3] = a;
				calib[4] = b;
				calib[5] = c;
				EEPROM.put(EEPROM_ADDR, calib);
				EEPROM.commit();
				fRunning = 0;
			}
		}
	}
}