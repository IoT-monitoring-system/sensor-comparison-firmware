#pragma once
#ifndef FSMANAGER_H
#define FSMANAGER_H

#include "FS.h"
#include <Arduino.h>

#include "FSFile.h"

class FSManager {
public:
  enum FileSystemType {
    FS_MANAGER_INVALID = 0,

    FS_MANAGER_LITTLE_FS,
    FS_MANAGER_SD,
  };

  FSManager();
  FSManager(fs::FS &fs, FileSystemType fsType);

  esp_err_t
  initializeFileSystem(fs::FS &fs, FileSystemType fsType);

  FileSystemType
  getFSType();

  void
  listDir(const std::string &dirname, uint8_t levels);

  void
  createDir(const std::string &path);

  void
  removeDir(const std::string &path);

  FSFile *
  readFile(const std::string &path);

  void
  writeFile(const std::string &path, const std::string &message);
  void
  writeFile(const std::string &path, FSFile &file);

  void
  appendFile(const std::string &path, const std::string &message);

  void
  renameFile(const std::string &path, const std::string &path2);

  void
  deleteFile(const std::string &path);

private:
  fs::FS *fileSystem;
  FSManager::FileSystemType fsType;
  bool
  isFileSystemValid();
};

#endif