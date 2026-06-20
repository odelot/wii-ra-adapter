/**********************************************************************************
 * PsramStream - PSRAM-backed buffer with the same interface as CharBufferStream.
 *
 * Identical to CharBufferStream (NES RA Adapter) except all heap operations
 * use heap_caps_* with MALLOC_CAP_SPIRAM so the large HTTP response buffer
 * lives in the 8 MB PSRAM instead of the ~250 KB internal heap.
 *
 * Used by gc-ra-adapter to receive and strip the RetroAchievements patch.php
 * response (~480 KB for SSBM) before passing the trimmed result to rcheevos.
 **********************************************************************************/

#ifndef PSRAM_STREAM_H
#define PSRAM_STREAM_H

#include <Arduino.h>
#include <Stream.h>
#include <esp_heap_caps.h>

class PsramStream : public Stream {
private:
  char*  _buffer;
  size_t _capacity;
  size_t _length;
  size_t _readPos;

public:
  PsramStream() : _buffer(nullptr), _capacity(0), _length(0), _readPos(0) {}

  ~PsramStream() { release(); }

  // Allocate or reallocate the buffer in PSRAM
  bool reserve(size_t size) {
    if (size == 0) { release(); return true; }

    if (_buffer == nullptr) {
      _buffer = (char*)heap_caps_malloc(size + 1,
                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (_buffer) {
        _buffer[0] = '\0';
        _capacity = size;
        _length   = 0;
        _readPos  = 0;
        return true;
      }
      return false;
    }

    char* nb = (char*)heap_caps_realloc(_buffer, size + 1,
                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (nb) {
      _buffer   = nb;
      _capacity = size;
      if (_length > size) { _length = size; _buffer[_length] = '\0'; }
      return true;
    }
    return false;
  }

  void release() {
    if (_buffer) { heap_caps_free(_buffer); _buffer = nullptr; }
    _capacity = _length = _readPos = 0;
  }

  bool shrink(size_t newSize) {
    if (newSize >= _capacity) return true;
    return reserve(newSize);
  }

  void clear() {
    _length = _readPos = 0;
    if (_buffer) _buffer[0] = '\0';
  }

  // Getters
  size_t      length()   const { return _length; }
  size_t      capacity() const { return _capacity; }
  const char* c_str()    const { return _buffer ? _buffer : ""; }
  char*       data()           { return _buffer; }

  // Stream write interface
  size_t write(uint8_t c) override {
    if (_buffer && _length < _capacity) {
      _buffer[_length++] = c;
      _buffer[_length]   = '\0';
      return 1;
    }
    return 0;
  }

  size_t write(const uint8_t* buf, size_t size) override {
    if (!_buffer) return 0;
    size_t toWrite = (size < (_capacity - _length)) ? size : (_capacity - _length);
    memcpy(_buffer + _length, buf, toWrite);
    _length += toWrite;
    _buffer[_length] = '\0';
    return toWrite;
  }

  // Stream read interface
  int available() override { return (int)(_length - _readPos); }
  int read()      override { return (_readPos < _length) ? (uint8_t)_buffer[_readPos++] : -1; }
  int peek()      override { return (_readPos < _length) ? (uint8_t)_buffer[_readPos]   : -1; }
  void flush()    override {}

  // Direct length update (after in-place edits)
  void setLength(size_t len) {
    if (len <= _capacity) { _length = len; if (_buffer) _buffer[_length] = '\0'; }
  }

  char  charAt(size_t i) const { return (_buffer && i < _length) ? _buffer[i] : '\0'; }
  char& operator[](size_t i)   { return _buffer[i]; }

  // Remove `count` bytes starting at `index` (in-place, no alloc)
  void removeRange(size_t index, size_t count) {
    if (!_buffer || index >= _length) return;
    if (index + count > _length) count = _length - index;
    memmove(_buffer + index, _buffer + index + count,
            _length - index - count + 1);  // +1 copies the '\0'
    _length -= count;
  }

  // Find substring (returns -1 if not found)
  int indexOf(const char* str, size_t from = 0) const {
    if (!_buffer || !str || from >= _length) return -1;
    const char* found = strstr(_buffer + from, str);
    return found ? (int)(found - _buffer) : -1;
  }

  int indexOf(char c, size_t from = 0) const {
    if (!_buffer || from >= _length) return -1;
    for (size_t i = from; i < _length; i++)
      if (_buffer[i] == c) return (int)i;
    return -1;
  }
};

#endif // PSRAM_STREAM_H
