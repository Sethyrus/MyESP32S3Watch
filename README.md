# MyESP32S3Watch

ESP32-S3-Touch-AMOLED-2.06

# Para flashear un binario

```
esptool --chip esp32-s3 \
  --port /dev/tty.usbmodem101 \
  --baud 921600 \
  --before default_reset --after hard_reset \
  write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect \
  0x0000 ./ESP32-S3-Touch-AMOLED-2.06-xiaozhi-251104.bin
```