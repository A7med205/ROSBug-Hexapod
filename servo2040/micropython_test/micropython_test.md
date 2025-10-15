### **1️⃣ Install MicroPython firmware onto the Servo2040**

1. **Put Servo2040 into bootloader mode**:

   * Unplug USB cable.
   * Hold down the **BOOTSEL** button on the board.
   * While holding it, plug the USB cable into your PC.
   * Release the button after it appears as a USB drive (should be called something like `RPI-RP2`).
2. **Copy the UF2 file**:

   * https://github.com/pimoroni/pimoroni-pico/blob/main/setting-up-micropython.md
   * Drag & drop your `.uf2` file onto the `RPI-RP2` drive.
   * The board will reboot automatically into MicroPython.

---

### **2️⃣ Connect to the board in Thonny**

1. Download https://thonny.org/
2. **Open Thonny**.
3. Go to **Tools → Options → Interpreter**.
4. Set:

   * **Interpreter** = *MicroPython (Raspberry Pi RP2040)* (or similar wording)
   * **Port** = The USB serial port for your Servo2040 (something like `/dev/ttyACM0` on Ubuntu).
5. Click **OK** — Thonny should now open a MicroPython REPL (you’ll see `>>>`).

---

### **3️⃣ Upload and run example scripts**

1. https://github.com/pimoroni/pimoroni-pico/tree/main/micropython/examples/servo2040
2. Open one of your example `.py` files in Thonny.
3. Click **Run** (green ▶).

   * If the example uses hardware like servos, make sure you have them connected correctly to the Servo2040.
4. If you want the example to run automatically on power-up:

   * Save it as `main.py` **to the board’s filesystem** (`File → Save as… → MicroPython device`).

---

### **4️⃣ Quick sanity check**

In the Thonny REPL (bottom window), try:

```python
import machine
print(machine.freq())
```

If you get a number (like `125000000`), MicroPython is running fine.

---

### **5️⃣ Troubleshooting tips**

* If Thonny can’t connect: check `dmesg | tail` after plugging in the board — make sure it appears as `/dev/ttyACM*`.
* If the examples throw errors: check if they require additional `.py` helper files — sometimes they come in a package.
* Make sure your UF2 file is specifically for **Servo2040**, not generic RP2040.
