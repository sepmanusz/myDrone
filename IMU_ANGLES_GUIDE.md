# IMU Szögszámítás - Drón Vezérléshez

## Áttekintés

Az új angle calculation modul 3 Euler-szöget számít ki az MPU9250 szenzorból:

- **Pitch (θ)**: Előre/hátra dőlés (Y tengely körüli forgás) - -90°...+90°
- **Roll (φ)**: Balra/jobbra dőlés (X tengely körüli forgás) - -180°...+180°
- **Yaw (ψ)**: Fejléc/irány (Z tengely körüli forgás) - 0°...360°

## Architektúra

### 1. **Accelerometer-alapú szögszámítás** (`IMU_CalculateAngles`)
Sztatikus helyzetből gyorsulásból számítja a szögeket:
```c
Roll  = atan2(accelY, accelZ)
Pitch = atan2(-accelX, sqrt(accelY² + accelZ²))
```

**Előnyök:**
- Nincs drift
- Hosszú távon pontosabb

**Hátrányok:**
- Megbízhatatlan mozgás közben (a gravitáción felüli gyorsulás nem mérhető)
- Lassú válaszidő

### 2. **Giroszkóp integráció** (`IMU_UpdateAnglesWithGyro`)
Az aktuális szöget frissíti a giroszkóp szögsebességével:
```c
angle_new = angle_old + gyro_rate × Δt
```

**Előnyök:**
- Nagyon gyors válaszidő
- Működik mozgás közben

**Hátrányok:**
- Drift: idővel eltérül az igazi értéktől

### 3. **Komplementer szűrő** (`IMU_ComplementaryFilter`) ⭐ JAVASOLT
Kombinál gyroszkópot (gyors) és gyorsulásmérőt (pontos):
```c
filtered_angle = α × (angle_old + gyro × Δt) + (1-α) × accel_angle
```

**Paraméter (α):**
- **0.98**: 98% giroszkóp, 2% accelerometer → dinamikus mozgáshoz
- **0.95**: 95% giroszkóp, 5% accelerometer → kiegyensúlyozott
- **0.90**: 90% giroszkóp, 10% accelerometer → statikus helyzethez

## Használat a Drón Vezérlésében

### 1. **Motor szabályozás**
```c
IMU_Angles_t angles;
Control_GetAngles(&angles);  // Szögek lekérdezése

// PID szabályozó pitch-hez
float pitch_error = target_pitch - angles.pitch;
motor_output = pid_pitch.Kp * pitch_error + ...
```

### 2. **Stabilizáció**
```c
// Szögek ellenőrzése - ha dől túl sokat, korrekcióval lejjebb
if (fabsf(angles.roll) > 45.0f) {
    // Vészhelyzeti leszállás
}
```

### 3. **Navigáció**
```c
// Irány (heading/yaw)
float current_yaw = angles.yaw;  // 0-360 fok
float desired_yaw = 90.0f;       // Kelet felé repülés

float yaw_error = desired_yaw - current_yaw;
// Normalize yaw_error to ±180
if (yaw_error > 180.0f) yaw_error -= 360.0f;
if (yaw_error < -180.0f) yaw_error += 360.0f;
```

## Control_Step funkcionalitás

Az `Control_Step` függvény automatikusan:
1. **Beolvassa** az MPU9250 szenzor adatait
2. **Kalibrálja** az offseteket
3. **Kiszámítja** a szögeket komplementer szűrővel
4. **Kiírja** az UART-ra az összes adatot (szögeket is)

**Kimeneti formátum:**
```
Angles: Pitch=12.3  Roll=-8.5  Yaw=275.2 deg
```

## Finomítás a Drónodhoz

### `control.c`-ben (6. sor körül):
```c
IMU_ComplementaryFilter(&current_angles, mpu_data, dt, 0.98f);
                                                          ^^^^
                                                  Ezt módosítsd!
```

**Tapasztalati értékek:**
- **0.99**: Agresszív, dinamikus drónnál
- **0.98**: Standard (repülőkhez)
- **0.95**: Lassú, stabil drónnál
- **0.90**: Stabil lebegéshez vagy drón szimulátorhoz

### Sampling Rate
Az alapértelmezett 100 Hz (10ms). Ha más a te samplingod:

```c
float dt = 0.01f;  // ← Ezt módosítsd! (másodperc)
// 50 Hz = 0.02f
// 200 Hz = 0.005f
```

## Tesztelés

### 1. UART kimenet ellenőrzése
Az `angles.pitch`, `angles.roll`, `angles.yaw` értékeket kövesd az UART-on.

### 2. Szögek validálása
```
Szobai teszt:
- Drón nyugalomban → pitch ≈ 0°, roll ≈ 0°, yaw = 0-360° (mágneses mező)
- 45° dőlés előre → pitch ≈ -45°
- 45° dőlés balra → roll ≈ -45°

Repülés közben:
- Drift ellenőrzése: 1 perc után drift < 5° legyen
```

### 3. Mágneses mérő kalibrálása
Az yaw és heading pontosságához jó mágneses kalibráció kell:
```c
// CALIBRATION_USAGE.md szerint kalibráld a mágnest
```

## API Referencia

### Függvények

#### `void IMU_CalculateAngles(const MPU9250_Data *mpu_data, IMU_Angles_t *angles)`
Gyorsulásmérőből és mágneses mérőből számítja a szögeket.
- **Használat**: Sztatikus helyzethez
- **Pontosság**: Kiváló (hosszú ideig)

#### `void IMU_UpdateAnglesWithGyro(IMU_Angles_t *angles, const MPU9250_Data *mpu_data, float dt)`
Giroszkóp szögsebességgel frissíti a szögeket.
- **Használat**: Gyors válaszidőhöz
- **Pontosság**: Rövid ideig jó, majd drift

#### `void IMU_ComplementaryFilter(IMU_Angles_t *angles, const MPU9250_Data *mpu_data, float dt, float alpha)`
Komplementer szűrő - **javasolt a drónhoz**.
- **Paraméterek**: `alpha` = 0.90-0.99
- **Pontosság**: Kiváló hosszú és rövid ideig

#### `void Control_GetAngles(IMU_Angles_t *angles)`
Az aktuális szögek lekérdezése a control loop-ból.

## Hibakeresés

### Probléma: Szögek nem frissülnek
```
→ Ellenőrizd: Control_Step meghívódik-e?
→ UART-on látod a szögeket?
→ MPU9250 inicializálódott-e?
```

### Probléma: Szögek szétszórtan/zajos
```
→ Növeld az alpha értéket (0.98 → 0.99)
→ Ellenőrizd az accelerometer kalibrációt
→ Mágneses mérő kalibrálása szükséges
```

### Probléma: Yaw nem helyes / mágneses mérő nem működik
```
→ Kalibráld a mágnest: CALIBRATION_USAGE.md
→ Budapest mágneses elhajlása: -4.0° ← ezt már beállítottuk
→ Fém részek távol a szenzortól
```

## Jövőbeli fejlesztések

### Madgwick szűrő
Még pontosabb, de több CPU szükséges.

### Kalman szűrő
Optimális, de bonyolult. Ha szükséges, Add hozzá később.

### Magnetometer drift korrekció
Az yaw hosszú ideig is stabil marad.

---

**Sikeres drón vezérléshez:**  
✅ Szögek jók → Motor kontrollerhez adjuk  
✅ Motor output helyes → Drón lebeg/szálló  
✅ PID hangolása → Stabil repülés
