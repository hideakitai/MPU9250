#pragma once
#ifndef EEPROMEXT_H
#define EEPROMEXT_H

#include <Arduino.h>
#include <EEPROM.h>

struct EEPROMExt : EEPROMClass
{
private:

    uint8_t* _data;
    size_t _size;
    bool _dirty;

public:

    uint8_t readByte (int address)
    {
        uint8_t value = 0;
        return readAll (address, value);
    }

    int8_t readChar (int address)
    {
        int8_t value = 0;
        return readAll (address, value);
    }

    uint8_t readUChar (int address)
    {
        uint8_t value = 0;
        return readAll (address, value);
    }

    int16_t readShort (int address)
    {
        int16_t value = 0;
        return readAll (address, value);
    }

    uint16_t readUShort (int address)
    {
        uint16_t value = 0;
        return readAll (address, value);
    }

    int32_t readInt (int address)
    {
        int32_t value = 0;
        return readAll (address, value);
    }

    uint32_t readUInt (int address)
    {
        uint32_t value = 0;
        return readAll (address, value);
    }

    int32_t readLong (int address)
    {
        int32_t value = 0;
        return readAll (address, value);
    }

    uint32_t readULong (int address)
    {
        uint32_t value = 0;
        return readAll (address, value);
    }

    int64_t readLong64 (int address)
    {
        int64_t value = 0;
        return readAll (address, value);
    }

    uint64_t readULong64 (int address)
    {
        uint64_t value = 0;
        return readAll (address, value);
    }

    float readFloat (int address)
    {
        float value = 0;
        return readAll (address, value);
    }

    double readDouble (int address)
    {
        double value = 0;
        return readAll (address, value);
    }

    bool readBool (int address)
    {
        int8_t value = 0;
        return readAll (address, value) ? 1 : 0;
    }

    size_t readString (int address, char* value, size_t maxLen)
    {
        if (!value)
            return 0;

        if (address < 0 || address + maxLen > _size)
            return 0;

        uint16_t len;
        for (len = 0; len <= _size; len++)
            if (_data[address + len] == 0)
            break;

        if (address + len > _size)
            return 0;

        memcpy((uint8_t*) value, _data + address, len);
        value[len] = 0;
        return len;
    }

    String readString (int address)
    {
        if (address < 0 || address > _size)
            return String(0);

        uint16_t len;
        for (len = 0; len <= _size; len++)
            if (_data[address + len] == 0)
            break;

        if (address + len > _size)
            return String(0);

        char value[len];
        memcpy((uint8_t*) value, _data + address, len);
        value[len] = 0;
        return String(value);
    }

    size_t readBytes (int address, void* value, size_t maxLen)
    {
        if (!value || !maxLen)
            return 0;

        if (address < 0 || address + maxLen > _size)
            return 0;

        memcpy((void*) value, _data + address, maxLen);
        return maxLen;
        }

        template <class T> T readAll (int address, T &value)
        {
        if (address < 0 || address + sizeof(T) > _size)
            return value;

        memcpy((uint8_t*) &value, _data + address, sizeof(T));
        return value;
    }

    template <class T> T writeAll (int address, const T &value)
    {
        if (address < 0 || address + sizeof(T) > _size)
            return value;

        memcpy(_data + address, (const uint8_t*) &value, sizeof(T));
        _dirty = true;

        return sizeof (value);
    }

    size_t writeByte (int address, uint8_t value)
    {
        return writeAll (address, value);
    }

    size_t writeChar (int address, int8_t value)
    {
        return writeAll (address, value);
    }

    size_t writeUChar (int address, uint8_t value)
    {
        return writeAll (address, value);
    }

    size_t writeShort (int address, int16_t value)
    {
        return writeAll (address, value);
    }

    size_t writeUShort (int address, uint16_t value)
    {
        return writeAll (address, value);
    }

    size_t writeInt (int address, int32_t value)
    {
        return writeAll (address, value);
    }

    size_t writeUInt (int address, uint32_t value)
    {
        return writeAll (address, value);
    }

    size_t writeLong (int address, int32_t value)
    {
        return writeAll (address, value);
    }

    size_t writeULong (int address, uint32_t value)
    {
        return writeAll (address, value);
    }

    size_t writeLong64 (int address, int64_t value)
    {
        return writeAll (address, value);
    }

    size_t writeULong64 (int address, uint64_t value)
    {
        return writeAll (address, value);
    }

    size_t writeFloat (int address, float value)
    {
        return writeAll (address, value);
    }

    size_t writeDouble (int address, double value)
    {
        return writeAll (address, value);
    }

    size_t writeBool (int address, bool value)
    {
        int8_t Bool;
        value ? Bool = 1 : Bool = 0;
        return writeAll (address, Bool);
    }

    size_t writeString (int address, const char* value)
    {
        if (!value)
            return 0;

        if (address < 0 || address > _size)
            return 0;

        uint16_t len;
        for (len = 0; len <= _size; len++)
            if (value[len] == 0)
            break;

        if (address + len > _size)
            return 0;

        memcpy(_data + address, (const uint8_t*) value, len + 1);
        _dirty = true;
        return strlen(value);
    }

    size_t writeString (int address, String value)
    {
        return writeString (address, value.c_str());
    }

    size_t writeBytes (int address, const void* value, size_t len)
    {
        if (!value || !len)
            return 0;

        if (address < 0 || address + len > _size)
            return 0;

        memcpy(_data + address, (const void*) value, len);
        _dirty = true;
        return len;
    }
};

#endif // EEPROMEXT_H
