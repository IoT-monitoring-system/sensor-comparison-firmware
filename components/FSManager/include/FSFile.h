#pragma once
#ifndef FSFILE_H
#define FSFILE_H

#include <string>

#include <Arduino.h>

class FSFile : public Stream {
public:
  struct file_data {
    uint32_t fileSize;
    uint8_t *byteBuffer;
  };

public:
  FSFile(const std::string &filePath, uint32_t size, uint8_t *buffer);
  FSFile(uint32_t size, uint8_t *buffer);
  ~FSFile();

  std::string
  getFileName() const;
  file_data *
  getFileData();
  bool
  isValid() const;

  // Stream methods implementation
  virtual int
  available() override;
  virtual int
  read() override;
  virtual size_t
  readBytes(uint8_t *buffer, size_t len) override;
  virtual int
  peek() override;
  virtual void
  flush() override;
  virtual size_t
  write(uint8_t byte) override;
  virtual size_t
  write(const uint8_t *buffer, size_t size) override;

private:
  std::string fileName;
  std::string filePath;
  file_data *file;

  size_t _index;
};

#endif