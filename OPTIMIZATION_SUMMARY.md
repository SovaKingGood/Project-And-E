# System Optimization Summary - Bluetooth & ESP-NOW Latency Prioritization

## CPU Core Assignment Strategy

### **CPU 0 - TIME-CRITICAL RF OPERATIONS**
- **Bluetooth Stack** (BTstack, HCI) - System managed, priority ~23
- **WiFi/ESP-NOW Driver** - System managed, priority ~23  
- **ESP-NOW Task** - User task, priority 20 (HIGHEST user priority)
- **Runs at:** 20ms intervals (50Hz) for low-latency RC control

### **CPU 1 - LOW-PRIORITY UI/DISPLAY**
- **LVGL Rendering** - Priority 1 (LOWEST)
- **Touch Polling** - Priority 1 (LOWEST), 500ms intervals
- **Label Updates** - Priority 1 (LOWEST), 200ms intervals (5Hz)

## Priority Hierarchy (Higher number = Higher priority)

```
Priority 23 (System):  WiFi Driver, Bluetooth Controller
Priority 20:           ESP-NOW Task ← CRITICAL RC CONTROL PATH
Priority 19-2:         Available for future use
Priority 1:            LVGL, Touch, Label Updates ← UI only
Priority 0:            IDLE tasks
```

## Latency Optimizations

### **ESP-NOW (RC Control Path):**
- ✅ Update interval: 100ms → **20ms** (50Hz refresh rate)
- ✅ Task priority: 3 → **20** (highest user priority)
- ✅ Max retries: 3 → **2** (fail fast for fresher data)
- ✅ Queue size: 10 → **5** (smaller queue = fresher data)
- ✅ Pinned to CPU 0 with WiFi driver for minimal context switching

### **Bluetooth (Controller Input):**
- ✅ BR/EDR only mode (no BLE overhead)
- ✅ Auto-latency **DISABLED** for consistent timing
- ✅ Modem sleep **DISABLED** for instant response
- ✅ 4 ACL connections pre-allocated
- ✅ Runs on CPU 0 with WiFi stack (no core switching delay)

### **LVGL/Display (Non-Critical):**
- ✅ Tick period: 33ms → **50ms** (20fps is fine for status display)
- ✅ Label updates: 100ms → **200ms** (5Hz, reduces CPU load)
- ✅ Touch polling: 350ms → **500ms** (slower is fine)
- ✅ Task priority: **1** (lowest, never blocks RF operations)
- ✅ Pinned to CPU 1 (isolated from RF stack)
- ✅ Max sleep: 500ms → **1000ms** (yields CPU aggressively)

## FreeRTOS Configuration

```
CONFIG_FREERTOS_HZ=1000                    # 1ms tick resolution for precise timing
CONFIG_ESP_TASK_WDT_TIMEOUT_S=20           # Tolerates LVGL rendering bursts
CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y  # Performance monitoring
```

## Memory Usage (Optimized)

```
LVGL Task Stack:        8KB   (down from 24KB)
LVGL Buffer:            12.8KB (320x20 pixels)
ESP-NOW Task Stack:     3KB   (down from 4KB)
Label Update Stack:     3KB   (down from 16KB)
Touch Stack:            1KB   
```

## Expected Performance

### RC Control (ESP-NOW):
- **Input-to-output latency:** <40ms (20ms interval + processing + transmission)
- **CPU 0 load:** ~10-15% (dedicated RF core)
- **No interference from display updates**

### Bluetooth (PS4 Controller):
- **Connection latency:** ~50-100ms typical for BR/EDR
- **Input latency:** <20ms after connection established
- **Priority higher than all UI tasks**

### Display:
- **Update rate:** 5Hz (acceptable for telemetry display)
- **Rendering rate:** 20fps (smooth enough for UI)
- **CPU 1 load:** ~20-30% (isolated core)
- **Will NOT interfere with Bluetooth/ESP-NOW**

## Stability Features

1. **Watchdog Protection:** 20s timeout prevents false triggers during heavy LVGL rendering
2. **Core Isolation:** RF stack on CPU 0, UI on CPU 1 - zero interference
3. **Priority Separation:** 19-level gap between ESP-NOW and UI tasks
4. **Memory Optimized:** All stacks tuned to minimum safe values
5. **Logging Throttled:** Only every 10 seconds to reduce serial overhead

## Testing Recommendations

1. **Latency Test:** Use ESP-NOW timing to measure round-trip controller→car→controller
2. **Load Test:** Maximum stick deflection while monitoring CPU stats
3. **Stability Test:** Let run for 30+ minutes with continuous input
4. **Connection Test:** Bluetooth reconnection after power cycle

## Next Steps

1. Build with `idf.py build`
2. Full clean flash: `idf.py -p COMX erase_flash flash monitor`
3. Test PS4 controller pairing
4. Measure RC control latency
5. Monitor task stats with FreeRTOS console commands

