#include "FSFile.h"

FSFile::FSFile(const std::string &filePath, uint32_t size, uint8_t *buffer)
    : fileName(filePath.substr(filePath.rfind('/') + 1)), filePath(filePath), _index(0) {
  file = new file_data();
  file->fileSize = size;

  if (size > 0 && buffer) {
    file->byteBuffer = buffer;
  } else {
    file->byteBuffer = nullptr;
  }
}

FSFile::FSFile(uint32_t size, uint8_t *buffer) {
  file = new file_data();
  file->fileSize = size;

  if (size > 0 && buffer) {
    file->byteBuffer = buffer;
  } else {
    file->byteBuffer = nullptr;
  }
}

FSFile::~FSFile() {
  if (file) {
    delete[] file->byteBuffer;
    delete file;
  }
}

std::string
FSFile::getFileName() const {
  return fileName;
}

FSFile::file_data *
FSFile::getFileData() {
  return file;
}

bool
FSFile::isValid() const {
  return (file && file->fileSize > 0 && file->byteBuffer != nullptr);
}

int
FSFile::available() {
  return file->fileSize - _index;
}

int
FSFile::read() {
  if (_index < file->fileSize) {
    return file->byteBuffer[_index++];
  }
  return -1;
}

size_t
FSFile::readBytes(uint8_t *buffer, size_t len) {
  size_t bytesRead = 0;
  while (bytesRead < len && _index < file->fileSize) {
    buffer[bytesRead++] = file->byteBuffer[_index++];
  }
  return bytesRead;
}

int
FSFile::peek() {
  if (_index < file->fileSize) {
    return file->byteBuffer[_index];
  }
  return -1;
}

void
FSFile::flush() {
  _index = 0;
}

size_t
FSFile::write(uint8_t byte) {
  return 0;
}
size_t
FSFile::write(const uint8_t *buffer, size_t size) {
  return 0;
}