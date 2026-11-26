// io.hpp - tolerant CSV dataset loader and small file helpers.
// Loading is O(total rows) time and O(V + E + rows) space.
#pragma once
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "chronos/graph.hpp"

namespace chronos {

struct LoadReport {
  int planets = 0, routeRows = 0, edges = 0, ships = 0, missions = 0, facilities = 0, events = 0;
  int duplicateEdges = 0, skippedRows = 0, unknownRefs = 0, missingFiles = 0;
  std::vector<std::string> warnings;
  double ms = 0;
};

// Splits one CSV line (supports double-quoted fields). O(len).
std::vector<std::string> splitCsvLine(const std::string& line);

// Header-aware CSV reader. Unknown/extra columns are ignored; missing columns
// fall back to defaults supplied by the caller.
class CsvReader {
 public:
  bool open(const std::string& path);
  bool next();                                   // advance to next non-empty data row
  int column(const std::string& name) const;     // -1 if absent (case-insensitive)
  std::string str(int col, const std::string& def = "") const;
  double num(int col, double def) const;         // non-numeric/missing -> def
  int64_t integer(int col, int64_t def) const;
  size_t rowNumber() const { return rowNo_; }

 private:
  std::vector<std::string> header_, row_;
  size_t rowNo_ = 0;
  std::unique_ptr<std::ifstream> in_;
};

// Loads planets/routes/spacecraft/missions/infrastructure/events/config CSVs from dir.
// Returns false only if no planets could be loaded.
bool loadDataset(const std::string& dir, World& world, LoadReport& report);

bool ensureDirectory(const std::string& dir);
bool writeTextFile(const std::string& path, const std::string& content);
std::string joinPath(const std::string& a, const std::string& b);

}  // namespace chronos
