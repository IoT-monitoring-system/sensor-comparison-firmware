#pragma once
#ifndef DATALOGGER_H
#define DATALOGGER_H
#include "FS.h"
#include <Arduino.h>

#include "FSFile.h"

enum file_system_type {
  FS_MANAGER_INVALID = 0,

  FS_MANAGER_LITTLE_FS,
  FS_MANAGER_SD,
};

class FSManager {
private:
  fs::FS *fileSystem;
  file_system_type fsType;

public:
  FSManager();
  FSManager(fs::FS &fs, file_system_type fsType);

  esp_err_t initializeFileSystem(fs::FS &fs, file_system_type fsType);

  file_system_type getFSType();

  void listDir(const String &dirname, uint8_t levels);

  void createDir(const String &path);

  void removeDir(const String &path);

  FSFile *readFile(const String &path);
  // void readFile(const char *path, );

  void writeFile(const String &path, const String &message);
  void writeFile(const String &path, FSFile &file);

  void appendFile(const String &path, const String &message);

  void renameFile(const String &path, const String &path2);

  void deleteFile(const String &path);

private:
  bool isFileSystemValid();
};

#endif