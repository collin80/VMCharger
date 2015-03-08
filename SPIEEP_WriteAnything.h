#if defined(ARDUINO) && ARDUINO < 100
  #include <WProgram.h>
#else
  #include <Arduino.h>
#endif

#include <SPIEEP.h>

template <class T> int eeWrite(SPIEEP& spieep, int ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++) {
    spieep.write(ee+i, *p++);
  }
  return i;
}

template <class T> int eeRead(SPIEEP& spieep, int ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++) {
    *p++ = spieep.read(ee+i);
  }
  return i;
}
