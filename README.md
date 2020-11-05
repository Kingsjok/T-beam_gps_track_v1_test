# T-beam_gps_track_v1_test
## Software dependencies
[TinyGPS++](http://arduiniana.org/libraries/tinygpsplus/)

[LMIC-Arduino](https://github.com/matthijskooijman/arduino-lmic)

## Payload decoder

```C
function Decoder(bytes, port) {
    var decoded = {};

    decoded.lat = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
    decoded.lat = (decoded.lat / 16777215.0 * 180) - 90;
  
    decoded.lon = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    decoded.lon = (decoded.lon / 16777215.0 * 360) - 180;
  
    var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
    var sign = bytes[6] & (1 << 7);
    if(sign)
    {
        decoded.alt = 0xFFFF0000 | altValue;
    }
    else
    {
        decoded.alt = altValue;
    }
  
    decoded.hdop = bytes[8] / 10.0;

    return decoded;
}
```
