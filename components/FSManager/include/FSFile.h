#pragma once
#ifndef FSFILE_H
#define FSFILE_H
#include <Arduino.h>

struct file_data {
  uint32_t fileSize;
  uint8_t *byteBuffer;
};

class FSFile : public Stream {
public:
  FSFile(const String &filePath, uint32_t size, uint8_t *buffer);
  FSFile(uint32_t size, uint8_t *buffer);
  ~FSFile();

  String getFileName() const;
  file_data *getFileData();
  bool isValid() const;

  // Stream methods implementation
  virtual int available() override;
  virtual int read() override;
  virtual size_t readBytes(uint8_t *buffer, size_t len) override;
  virtual int peek() override;
  virtual void flush() override;
  virtual size_t write(uint8_t byte) override;
  virtual size_t write(const uint8_t *buffer, size_t size) override;

private:
  String fileName;
  String filePath;
  file_data *file;

  size_t _index;
};

#endif