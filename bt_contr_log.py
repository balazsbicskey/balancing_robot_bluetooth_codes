import serial
import asyncio
import csv
import time

class Balanced:
    def __init__(self):
        self.kp_balance = 38.0
        self.kd_balance = 0.60
        self.kp_speed = 10.0
        self.ki_speed = 0.25
        self.kp_turn = 2.5
        self.kd_turn = 0.5

        self.pwm_left = 0.0
        self.pwm_right = 0.0

        self.encoder_left = 0.0
        self.encoder_right = 0.0

        self.interrupt_cnt = 0
        self.speed_integral = 0.0
        self.speed_filter_old = 0.0

    def pd_vertical_ring(self, angle, gyro_x):
        # angle/gyroscope offset benne marad, ahogy eddig használtad
        return self.kp_balance * (angle + 3.5) + self.kd_balance * (gyro_x - 0)

    def pi_speed_ring(self):
        car_speed = (self.encoder_left + self.encoder_right) * 0.5
        self.encoder_left = 0
        self.encoder_right = 0

        speed_f = self.speed_filter_old * 0.7 + car_speed * 0.3
        self.speed_filter_old = speed_f

        self.speed_integral += speed_f
        self.speed_integral = max(min(self.speed_integral, 3000), -3000)

        return -(self.kp_speed * speed_f + self.ki_speed * self.speed_integral)

    def pi_turn_ring(self, gyro_z):
        return self.kd_turn * gyro_z

    def compute(self, angle, gyro_x, gyro_z, speedL, speedR):
        self.encoder_left += speedL
        self.encoder_right += speedR
        
        balance = self.pd_vertical_ring(angle, gyro_x)
        self.interrupt_cnt += 1

        if self.interrupt_cnt >= 8:
            self.interrupt_cnt = 0
            speed = self.pi_speed_ring()
            turn = self.pi_turn_ring(gyro_z)
        else:
            speed = 0
            turn = 0

        pwmL = balance - speed - turn
        pwmR = balance - speed + turn

        pwmL = max(min(pwmL, 255), -255)
        pwmR = max(min(pwmR, 255), -255)

        return pwmL, pwmR


# ide gyűjtjük az i, angle+3.5 párokat
angle_log = []

# ha ritkítani akarod a logot, ezt állíthatod (1 = minden mintát logol)
LOG_EVERY_N = 1


async def main_loop():
    print("Controller started. Connecting to /dev/rfcomm0...")
    bt = serial.Serial('/dev/rfcomm0', 115200, timeout=0.005, write_timeout=0.001)
    await asyncio.sleep(1)

    bal = Balanced()
    sample_count = 0

    while True:
        try:
            line = bt.readline().decode(errors="ignore").strip()
            if not line.startswith("A"):
                continue  

            # Incoming robot telemetry:
            # Format: A angle,iteration G gyroX,gyroZ S speedL,speedR
            parts = line.split(" ")

            angle = float(parts[0][2:].split(",")[0])
            i     = int(parts[0][2:].split(",")[1])

            gyroX = float(parts[1][2:].split(",")[0])
            gyroZ = float(parts[1][2:].split(",")[1])

            speedL = float(parts[2][2:].split(",")[0])
            speedR = float(parts[2][2:].split(",")[1])

            pwmL, pwmR = bal.compute(angle, gyroX, gyroZ, speedL, speedR)

            # Send motor output BACK to transmitter
            out = f"L:{pwmL:.2f} R:{pwmR:.2f} I:{i}\n"
            bt.write(out.encode())

            # --- NINCS print, csak memória-log ---
            sample_count += 1
            if sample_count % LOG_EVERY_N == 0:
                # ugyanazt logoljuk, amit eddig kiírtál: angle + 3.5
                angle_corr = angle + 3.5
                angle_log.append((i, angle_corr))

        except Exception as e:
            # ritka hiba esetén még mindig kiírunk, ez nem lassít folyamatosan
            print("Controller error:", e)
            await asyncio.sleep(0.01)


def save_angle_log(filename="angle_log.csv"):
    if not angle_log:
        print("No angle data to save.")
        return
    try:
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["i", "angle_plus_3.5"])
            writer.writerows(angle_log)
        print(f"Saved {len(angle_log)} samples to {filename}")
    except Exception as e:
        print("Failed to save angle log:", e)


if __name__ == "__main__":
    try:
        asyncio.run(main_loop())
    except KeyboardInterrupt:
        print("\nStopping controller (KeyboardInterrupt).")
    finally:
        # futás végén log mentése
        save_angle_log()

