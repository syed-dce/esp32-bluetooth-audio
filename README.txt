

========================================== BUILD and FLASH ==========================================

1. Navigate to project directory

2. idf.py set-target esp32s3

3. idf.py build

4. idf.py -p /dev/ttyUSB0 flash monitor		// replace with appropriate PORT for your device

========================================== USAGE ==========================================

1. Install USB serial on android device

2. Connect with esp32

3. Set baudrate in the mobile app as 115200

4. Start viewing "Msg from ESP" on the app

5. Send any data from app to ESP32 and view it on computer.
