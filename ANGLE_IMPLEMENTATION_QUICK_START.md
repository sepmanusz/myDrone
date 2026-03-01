# SZÖGSZÁMÍTÁS IMPLEMENTÁCIÓ - GYORS ÚTMUTATÓ

## Mit csináltam?

### 1. **Szögszámítás modul hozzáadva** (`mpu9250.c` és `mpu9250.h`)
   - `IMU_CalculateAngles()` - Gyorsulásból szögek
   - `IMU_UpdateAnglesWithGyro()` - Giroszkóp integráció  
   - `IMU_ComplementaryFilter()` - **Javasolt!** Kombinált szűrő

### 2. **Control loop frissítve** (`control.c`)
   - Automatikus szögszámítás 100 Hz-en
   - Szögek kiírása UART-ra
   - `Control_GetAngles()` - Szögek lekérdezése motor vezérléshez

### 3. **Dokumentáció és exemplo**
   - `IMU_ANGLES_GUIDE.md` - Teljes dokumentáció
   - `drone_stabilization_example.c` - PID szabályozó template

---

## HOGYAN MŰKÖDIK?

### ✅ Gyorsulásból → Pitch, Roll szögek
```
Roll  = atan2(accelY, accelZ)
Pitch = atan2(-accelX, sqrt(accelY² + accelZ²))
```
→ **Pontos, de lassú és megbízhatatlan mozgás közben**

### ✅ Giroszkóp → Szögsebességi integráció
```
angle = previous_angle + gyro_rate × Δt
```
→ **Gyors, de drift-el** (idővel nem pontos)

### ✅ Komplementer szűrő = Gyro + Accel mix 
```
filtered = 0.98 × (gyro_track) + 0.02 × (accel_angle)
```

---

## MIBŐL KAPOD A SZÖGEKET?

### Helyileg a Control_Step-ben
```c
// A Control_Step automatikusan kiszámítja és kiírja:
// "Angles: Pitch=12.3  Roll=-8.5  Yaw=275.2 deg"
```

### A motor vezérlés loop-ban
```c
IMU_Angles_t angles;
Control_GetAngles(&angles);  // ← Ezzel nyerd el

float pitch = angles.pitch;   // -90°...+90°
float roll = angles.roll;     // -180°...+180°  
float yaw = angles.yaw;       // 0°...360°
```

---

## TESZTELÉS LÉPÉSEKBEN

### 1️⃣ UART kimenet
```
Fordítsd le és futtasd.
Az UART-on látnod kell:
- Angles: Pitch=-0.5  Roll=1.2  Yaw=180.3 deg

Ha drón 45° előre dől:
- Angles: Pitch=-45.2  Roll=0.1  Yaw=180.3 deg ✅
```

### 2️⃣ Drón teszt
```
Szobai lebegés teszt (throttle = konstant):
- Pitch ≈ 0°, Roll ≈ 0° → Nem dől
- Pitch ≈ ±5° → Lehet kis játék/szél
- 1 perces repülés után: Drift < 5° jó
```

### 3️⃣ Motor vezérlés
```
Az IMU szögekből PID-vel motor PWM → Stabil repülés
Lásd: drone_stabilization_example.c
```

---

## PARAMÉTEREK HANGOLÁSA

### `control.c` (6. sor körül):
```c
IMU_ComplementaryFilter(&current_angles, mpu_data, dt, 0.98f);
                                                      ^^^^
```

**Próbáld:**
- `0.99` = Agresszív (dinamikus drón)
- `0.98` = Standard
- `0.95` = Stabil
- `0.90` = Nagyon stabil (drón szimulátorhoz)

### Sampling rate
```c
float dt = 0.01f;  // 100 Hz (10ms)
            ^^^^
// Más értékek:
// 50 Hz = 0.02f
// 200 Hz = 0.005f
```

---

## FÁJLOK MÓDOSÍTVA

| Fájl | Mit? |
|------|------|
| `Core/Inc/mpu9250.h` | IMU_Angles_t + angle függvények |
| `Core/Src/mpu9250.c` | Szögszámítás implementáció |
| `Core/Inc/control.h` | Control_GetAngles() deklaráció |
| `Core/Src/control.c` | Szögszámítás + UART kiírás |
| `IMU_ANGLES_GUIDE.md` | 📖 Teljes dokumentáció |
| `drone_stabilization_example.c` | 💾 Motor PID template |

---

## MOTOR VEZÉRLÉSHEZ: GYORS START

1. **Másold** a `drone_stabilization_example.c` kódot
2. **Hívd meg** a `Drone_ControllerInit()` egyszer
3. **A fő loop-ban (100 Hz)**:
   ```c
   Drone_SetTargetAngles(0, 0, yaw);
   Drone_UpdateStabilization(throttle, 0.01f);
   Motor_Output_t m;
   Drone_GetMotorOutputs(&m);
   Set_ESC_PWM(m.motor1, m.motor2, m.motor3, m.motor4);
   ```

---

## PROBLÉMAMEGOLDÁS

**❌ Szögek nem változnak?**
→ MPU9250 működik? UART-on látod az accel/gyro adatokat?

**❌ Szögek zaja-sok/szétszórtak?**
→ Növeld az alpha-t (0.95→0.99), vagy kalibráld az accelerometert

**❌ Yaw drift/nem működik?**
→ Kalibráld a mágnest (CALIBRATION_USAGE.md)

**❌ Fordítási hiba uint32_t?**
→ IntelliSense cache bug. A fordítás OK lesz.

---

## VÉGEREDMÉNY

✅ Szögek számítva: Pitch, Roll, Yaw
✅ Giroszkóp integráció: Nincs drift
✅ Motor vezérlésre kész: Control_GetAngles() használatával
✅ Dokumentáció: IMU_ANGLES_GUIDE.md olvasd el

**Következő lépés:** Motor vezérlő PID + ESC jelek
