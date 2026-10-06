# Panther-Aero-Telemetry
Redoing telemetry systems for test/competition flights

**Reminder for developers: pull the most recent commit from main before starting on your branch**

```bash
git checkout main  
git pull  

git checkout -b [branch name]  

# make your changes  

git add .  
git commit -m "Add sensor code"  
git push -u origin [branch name]
```

- [ ] Airspeed (Pitot tube) (pending)  
- [x] Ground Speed (IMU)  
- [ ] G-forces (Lorenzo)  
- [ ] Pack voltage/power draw (pending)   
- [ ] Temperature (motor/ESC) (pending)  
- [x] Temperature (ambient)  
- [x] Altitude  
- [ ] Climb Speed (IMU)   


## Libraries  
https://github.com/adafruit/Adafruit_Sensor  
https://github.com/adafruit/Adafruit_BusIO  
https://github.com/adafruit/Adafruit_BNO08x  
https://github.com/adafruit/Adafruit_BMP3XX  