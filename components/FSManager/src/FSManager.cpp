#include "esp_log.h"

#include "FSManager.h"

static const char *TAG = "FSManager";

FSManager::FSManager() : fileSystem(NULL) {
}
FSManager::FSManager(fs::FS &fs, FileSystemType fsType) : fileSystem(&fs), fsType(fsType) {
}

bool
FSManager::isFileSystemValid() {
  return (fsType > 0) && (fileSystem);
}

esp_err_t
FSManager::initializeFileSystem(fs::FS &fs, FileSystemType fsType) {
  if (isFileSystemValid()) {
    ESP_LOGE(TAG, "File system is already initialized%s", "");
    return ESP_FAIL;
  }

  fileSystem = &fs;
  this->fsType = fsType;

  return ESP_OK;
}

FSManager::FileSystemType
FSManager::getFSType() {
  if (isFileSystemValid()) {
    ESP_LOGE(TAG, "File system is already initialized%s", "");
    return FS_MANAGER_INVALID;
  }

  return fsType;
}

void
FSManager::listDir(const std::string &dirname, uint8_t levels) {
  if (!isFileSystemValid()) {
    ESP_LOGE(TAG, "File system is not initialized%s", "");
    return;
  }

  File root = fileSystem->open(dirname.c_str());
  if (!root) {
    ESP_LOGE(TAG, "Failed to open directory%s", "");
    return;
  }
  if (!root.isDirectory()) {
    ESP_LOGE(TAG, "Not a directory%s", "");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      ESP_LOGD(TAG, "  DIR : %s", file.name());
      if (levels) {
        listDir(file.path(), levels - 1);
      }
    } else {
      ESP_LOGD(TAG, "  FILE: %s", file.name());
      ESP_LOGD(TAG, "  SIZE: %u", file.size());
    }
    file = root.openNextFile();
  }
}

void
FSManager::createDir(const std::string &path) {
  if (!isFileSystemValid()) {
    ESP_LOGE(TAG, "File system is not initialized%s", "");
    return;
  }

  if (!fileSystem->mkdir(path.c_str()))
    ESP_LOGE(TAG, "mkdir failed%s", "");
}

void
FSManager::removeDir(const std::string &path) {
  if (!isFileSystemValid()) {
    ESP_LOGE(TAG, "File system is not initialized%s", "");
    return;
  }

  if (!fileSystem->rmdir(path.c_str()))
    ESP_LOGE(TAG, "rmdir failed%s", "");
}

FSFile *
FSManager::readFile(const std::string &path) {
  if (!isFileSystemValid()) {
    ESP_LOGE(TAG, "File system is not initialized%s", "");
    return nullptr;
  }

  File file = fileSystem->open(path.c_str());
  if (!file) {
    ESP_LOGE(TAG, "Failed to open file for reading");
    return nullptr;
  }

  uint32_t fileSize = file.size();
  if (fileSize == 0) {
    ESP_LOGE(TAG, "File is empty: %s", path.c_str());
    file.close();
    return nullptr;
  }

  uint8_t *buffer = new uint8_t[fileSize];
  if (!buffer) {
    ESP_LOGE(TAG, "Memory allocation failed!");
    file.close();
    return nullptr;
  }

  file.read(buffer, fileSize);
  file.close();

  FSFile *fsFile = new FSFile(path, fileSize, buffer);

  return fsFile;
}

void
FSManager::writeFile(const std::string &path, const std::string &message) {
  if (!isFileSystemValid()) {
    ESP_LOGE(TAG, "File system is not initialized%s", "");
    return;
  }

  File file = fileSystem->open(path.c_str(), FILE_WRITE);
  if (!file) {
    ESP_LOGE(TAG, "Failed to open file for writing%s", "");
    return;
  }
  if (!file.print(message.c_str()))
    ESP_LOGE(TAG, "Write failed%s", "");

  file.close();
}

void
FSManager::writeFile(const std::string &path, FSFile &file) {
  if (!isFileSystemValid()) {
    ESP_LOGE(TAG, "File system is not initialized%s", "");
    return;
  }

  if (!file.isValid()) {
    ESP_LOGE(TAG, "Invalid FSFile: Nothing to write");
    return;
  }

  File outFile = fileSystem->open((path + file.getFileName()).c_str(), FILE_WRITE);
  if (!outFile) {
    ESP_LOGE(TAG, "Failed to open file for writing: %s", file.getFileName().c_str());
    return;
  }

  size_t bytesWritten = outFile.write(file.getFileData()->byteBuffer, file.getFileData()->fileSize);
  if (bytesWritten != file.getFileData()->fileSize) {
    ESP_LOGE(TAG, "Failed to write full file: %s", file.getFileName().c_str());
  } else {
    ESP_LOGD(TAG, "File written successfully: %s", file.getFileName().c_str());
  }

  outFile.close();
}

void
FSManager::appendFile(const std::string &path, const std::string &message) {
  if (!isFileSystemValid()) {
    ESP_LOGE(TAG, "File system is not initialized%s", "");
    return;
  }

  File file = fileSystem->open(path.c_str(), FILE_APPEND);
  if (!file) {
    ESP_LOGE(TAG, "Failed to open file for appending%s", "");
    return;
  }
  if (!file.print(message.c_str()))
    ESP_LOGE(TAG, "Append failed%s", "");

  file.close();
}

void
FSManager::renameFile(const std::string &path1, const std::string &path2) {
  if (!isFileSystemValid()) {
    ESP_LOGE(TAG, "File system is not initialized%s", "");
    return;
  }

  if (!fileSystem->rename(path1.c_str(), path2.c_str()))
    ESP_LOGE(TAG, "Rename failed%s", "");
}

void
FSManager::deleteFile(const std::string &path) {
  if (!isFileSystemValid()) {
    ESP_LOGE(TAG, "File system is not initialized%s", "");
    return;
  }

  if (!fileSystem->remove(path.c_str()))
    ESP_LOGE(TAG, "Delete failed%s", "");
}