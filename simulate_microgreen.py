# simulate_microgreen.py
# -*- coding: utf-8 -*-
import math
import time
import random
import datetime as dt

# ======== กติกาควบคุม (ปรับได้) ========
FAN_ON_RANGE = (15.0, 30.0)     # 15–30 °C → พัดลม ON, นอกช่วง → OFF
PUMP_ON_THRESHOLD = 70.0        # Soil < 70% → ปั๊ม ON, ≥70% → OFF
LIGHT_DELAY_DAYS = 3            # ไฟปลูก: เริ่ม OFF และ "จะเปิด" ในอีก N วัน

PRINT_EVERY_SEC = 2             # พิมพ์ทุกกี่วินาที
# ========================================

# ช่วยพิมพ์เวลานับถอยหลัง
def eta_txt(t: dt.datetime):
    now = dt.datetime.now()
    if t <= now:
        return "กำหนดถึงเวลาแล้ว"
    d = t - now
    days = d.days
    h = d.seconds // 3600
    m = (d.seconds % 3600) // 60
    return f"อีก {days} วัน {h} ชม. {m} นาที"

class GreenhouseSim:
    """
    โมเดลอย่างง่ายแบบ mean-reverting + ความสัมพันธ์
    - Temp ถูกดึงเข้าหา temp_target (ที่ค่อย ๆ เปลี่ยนช้า ๆ ตามไซน์เวฟ/สุ่มเบา ๆ)
    - ความชื้นอากาศสวนทางกับ Temp เล็กน้อย และเพิ่มเล็กน้อยตอนปั๊ม ON
    - Soil moisture ค่อย ๆ แห้ง (evaporation) เมื่อปั๊ม OFF และค่อย ๆ เพิ่มเมื่อปั๊ม ON
    - มี noise เล็ก ๆ เพื่อไม่ให้เนียนเกินไป
    """

    def __init__(self):
        # สภาวะเริ่มต้น (ค่า realistic)
        self.temp = 26.0         # °C
        self.rh_air = 65.0       # %RH
        self.soil = 68.0         # % (0 แห้งสนิท, 100 เปียกมาก)

        # เป้าหมายพื้นฐานกลางวัน/กลางคืน (จำลอง 24h ด้วยไซน์เวฟช้า ๆ)
        self.t0 = time.time()
        self.day_amplitude = 4.0       # ช่วงแกว่งกลางวัน/กลางคืน ~±4°C
        self.base_temp = 25.0          # จุดกึ่งกลาง
        self.base_rh = 60.0            # RH พื้นฐาน

        # ค่าพารามิเตอร์การเปลี่ยนแปลง (ปรับได้)
        self.alpha_temp = 0.07         # ความเร็วดึงกลับอุณหภูมิ
        self.alpha_rh = 0.1            # ความเร็วดึงกลับ RH
        self.evap_rate = 0.20          # ดินแห้งลง/นาที (เมื่อปั๊ม OFF) หน่วย %/min
        self.irrig_rate = 0.60         # ดินชื้นขึ้น/นาที (เมื่อปั๊ม ON) หน่วย %/min

        # noise
        self.temp_noise = 0.08         # °C ต่อรอบ
        self.rh_noise = 0.15           # % ต่อรอบ
        self.soil_noise = 0.1          # % ต่อรอบ (น้อยมาก)

        # ผลลัพธ์ actuator
        self.fan = "OFF"
        self.pump = "OFF"

        # เวลาจะเปิดไฟปลูก
        self.light_on_at = dt.datetime.now() + dt.timedelta(days=LIGHT_DELAY_DAYS)

    def _day_cycle_temp_target(self):
        # ไซน์เวฟ 24 ชม.: 2π * (ชั่วโมง/24)
        hours = (time.time() - self.t0) / 3600.0
        return self.base_temp + self.day_amplitude * math.sin(2 * math.pi * hours / 24.0)

    def step(self, dt_sec: float):
        # ---- 1) ตัดสินใจ actuator จากค่าปัจจุบัน ----
        self.fan = "ON" if (FAN_ON_RANGE[0] <= self.temp <= FAN_ON_RANGE[1]) else "OFF"
        self.pump = "ON" if (self.soil < PUMP_ON_THRESHOLD) else "OFF"

        # ---- 2) คำนวณเป้าหมาย/ดึงกลับ ----
        temp_target = self._day_cycle_temp_target()
        # เพิ่มเทอมสวนทางกับ RH เล็ก ๆ (ยิ่งร้อน RH จะมีแนวโน้มต่ำลง)
        rh_target = self.base_rh - 0.4 * (self.temp - self.base_temp)
        # ถ้าปั๊ม ON ความชื้นอากาศมักเพิ่มเล็กน้อย
        if self.pump == "ON":
            rh_target += 2.0

        # ---- 3) อัปเดตค่าแบบ mean-reverting + noise เล็กน้อย ----
        # อุณหภูมิ
        self.temp += self.alpha_temp * (temp_target - self.temp) + random.uniform(-self.temp_noise, self.temp_noise)
        self.temp = max(10.0, min(40.0, self.temp))  # ขอบเขตใช้งาน DHT11

        # RH อากาศ
        self.rh_air += self.alpha_rh * (rh_target - self.rh_air) + random.uniform(-self.rh_noise, self.rh_noise)
        self.rh_air = max(30.0, min(95.0, self.rh_air))

        # ความชื้นดิน: เพิ่มหรือลดตามสถานะปั๊ม (แปลงอัตราต่อ "นาที" → ต่อ dt_sec)
        if self.pump == "ON":
            self.soil += self.irrig_rate * (dt_sec / 60.0)
        else:
            self.soil -= self.evap_rate * (dt_sec / 60.0)

        # ใส่ noise เบา ๆ + จำกัดช่วง
        self.soil += random.uniform(-self.soil_noise, self.soil_noise)
        self.soil = max(10.0, min(95.0, self.soil))

    def light_status(self):
        return "ON" if dt.datetime.now() >= self.light_on_at else "OFF"

    def light_note(self):
        if self.light_status() == "ON":
            return "(เริ่มเปิดแล้วหรือถึงกำหนด)"
        return f"(จะเปิดประมาณ {self.light_on_at.strftime('%Y-%m-%d %H:%M')} — {eta_txt(self.light_on_at)})"

def main():
    sim = GreenhouseSim()
    print("=== Microgreen Controller Simulation (Realistic) ===")
    print(f"- กฎพัดลม : {FAN_ON_RANGE[0]}–{FAN_ON_RANGE[1]} °C → ON")
    print(f"- กฎปั๊ม  : Soil < {PUMP_ON_THRESHOLD:.0f}% → ON, ≥{PUMP_ON_THRESHOLD:.0f}% → OFF")
    print(f"- ไฟปลูก  : OFF ก่อน และจะเปิดในอีก {LIGHT_DELAY_DAYS} วัน")
    print("กด Ctrl+C เพื่อหยุด\n")

    tick = 0
    while True:
        tick += 1
        start = time.time()

        # 1 step
        sim.step(PRINT_EVERY_SEC)

        # แสดงผลสวย ๆ ไม่กระโดด
        now = dt.datetime.now().strftime("%H:%M:%S")
        print(
            f"[{now}] รอบ {tick:>4d} | "
            f"🌡 Temp {sim.temp:>5.1f}°C | "
            f"💧 Air {sim.rh_air:>5.1f}% | "
            f"🪴 Soil {sim.soil:>5.1f}% | "
            f"🧊 Fan {sim.fan:<3s} | "
            f"🚰 Pump {sim.pump:<3s} | "
            f"💡 Light {sim.light_status()} {sim.light_note()}"
        )

        # หน่วงเวลาให้คงที่
        elapsed = time.time() - start
        sleep = max(0.0, PRINT_EVERY_SEC - elapsed)
        time.sleep(sleep)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nหยุดการจำลองแล้ว 👍")
 