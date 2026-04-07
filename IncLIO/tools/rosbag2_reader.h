#ifndef INCLIO_ROSBAG2_READER_H
#define INCLIO_ROSBAG2_READER_H

#include <sqlite3.h>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace rosbag2 {

// ---------------------------------------------------------------------------
// CDR (Common Data Representation) reader for ROS2 serialized messages.
// Assumes little-endian CDR (the default on all ROS2 platforms).
// ---------------------------------------------------------------------------
class CdrReader {
   public:
    CdrReader(const uint8_t* data, size_t size) : data_(data), size_(size), pos_(4) {
        // First 4 bytes are the CDR encapsulation header.
        // Byte 1: 0x01 = CDR Little Endian
        if (size < 4 || data[1] != 0x01) {
            throw std::runtime_error("Not a CDR-LE serialized message");
        }
    }

    void align(size_t a) {
        // Alignment is relative to CDR body start (after 4-byte encapsulation header)
        size_t rem = (pos_ - 4) % a;
        if (rem) pos_ += a - rem;
    }

    int32_t read_int32() {
        align(4);
        int32_t v;
        std::memcpy(&v, data_ + pos_, 4);
        pos_ += 4;
        return v;
    }
    uint32_t read_uint32() {
        align(4);
        uint32_t v;
        std::memcpy(&v, data_ + pos_, 4);
        pos_ += 4;
        return v;
    }
    uint64_t read_uint64() {
        align(8);
        uint64_t v;
        std::memcpy(&v, data_ + pos_, 8);
        pos_ += 8;
        return v;
    }
    uint8_t read_uint8() { return data_[pos_++]; }
    float read_float32() {
        align(4);
        float v;
        std::memcpy(&v, data_ + pos_, 4);
        pos_ += 4;
        return v;
    }
    double read_float64() {
        align(8);
        double v;
        std::memcpy(&v, data_ + pos_, 8);
        pos_ += 8;
        return v;
    }
    std::string read_string() {
        uint32_t len = read_uint32();  // includes null terminator
        std::string s(reinterpret_cast<const char*>(data_ + pos_), len > 0 ? len - 1 : 0);
        pos_ += len;
        return s;
    }
    const uint8_t* read_raw(size_t n) {
        const uint8_t* p = data_ + pos_;
        pos_ += n;
        return p;
    }
    void skip(size_t n) { pos_ += n; }
    size_t position() const { return pos_; }

   private:
    const uint8_t* data_;
    size_t size_;
    size_t pos_;
};

// ---------------------------------------------------------------------------
// ROS2 std_msgs/Header parsing helper
// ---------------------------------------------------------------------------
struct RosTime {
    int32_t sec = 0;
    uint32_t nanosec = 0;
    double to_sec() const { return sec + nanosec * 1e-9; }
};

inline RosTime read_header(CdrReader& r) {
    RosTime t;
    t.sec = r.read_int32();
    t.nanosec = r.read_uint32();
    /*frame_id=*/r.read_string();
    return t;
}

// ---------------------------------------------------------------------------
// Topic metadata
// ---------------------------------------------------------------------------
struct TopicInfo {
    int id = 0;
    std::string name;
    std::string type;
    std::string serialization_format;
};

// ---------------------------------------------------------------------------
// A single raw message from the bag
// ---------------------------------------------------------------------------
struct RawMessage {
    int64_t timestamp_ns = 0;  // nanoseconds since epoch
    int topic_id = 0;
    std::vector<uint8_t> data;
};

// ---------------------------------------------------------------------------
// SQLite3-based ROS2 bag reader.
//
// Supports the default "sqlite3" storage plugin format used by rosbag2.
// Bag path can be either:
//   - The directory containing metadata.yaml + .db3 files
//   - A .db3 file directly
// ---------------------------------------------------------------------------
class BagReader {
   public:
    BagReader() = default;
    ~BagReader() { close(); }

    BagReader(const BagReader&) = delete;
    BagReader& operator=(const BagReader&) = delete;

    void open(const std::string& path) {
        namespace fs = std::filesystem;
        std::string db_path;

        if (fs::is_directory(path)) {
            // Look for .db3 files in the bag directory
            for (const auto& entry : fs::directory_iterator(path)) {
                if (entry.path().extension() == ".db3") {
                    db_path = entry.path().string();
                    break;
                }
            }
            if (db_path.empty()) {
                throw std::runtime_error("No .db3 file found in bag directory: " + path);
            }
        } else if (fs::exists(path)) {
            db_path = path;
        } else {
            throw std::runtime_error("Bag path does not exist: " + path);
        }

        int rc = sqlite3_open_v2(db_path.c_str(), &db_, SQLITE_OPEN_READONLY, nullptr);
        if (rc != SQLITE_OK) {
            throw std::runtime_error("Cannot open bag: " + std::string(sqlite3_errmsg(db_)));
        }

        load_topics();
    }

    void close() {
        if (db_) {
            sqlite3_close(db_);
            db_ = nullptr;
        }
    }

    const std::map<int, TopicInfo>& topics() const { return topics_; }

    // Find a topic by name. Returns nullptr if not found.
    const TopicInfo* find_topic(const std::string& name) const {
        for (const auto& [id, ti] : topics_) {
            if (ti.name == name) return &ti;
        }
        return nullptr;
    }

    // Iterate all messages ordered by timestamp, calling fn for each.
    // If topic_ids is non-empty, only those topics are returned.
    void for_each_message(const std::vector<int>& topic_ids,
                          const std::function<void(const RawMessage&)>& fn) const {
        std::string sql = "SELECT topic_id, timestamp, data FROM messages";
        if (!topic_ids.empty()) {
            sql += " WHERE topic_id IN (";
            for (size_t i = 0; i < topic_ids.size(); i++) {
                if (i > 0) sql += ",";
                sql += std::to_string(topic_ids[i]);
            }
            sql += ")";
        }
        sql += " ORDER BY timestamp ASC";

        sqlite3_stmt* stmt = nullptr;
        int rc = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
        if (rc != SQLITE_OK) {
            throw std::runtime_error("SQL prepare failed: " + std::string(sqlite3_errmsg(db_)));
        }

        RawMessage msg;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            msg.topic_id = sqlite3_column_int(stmt, 0);
            msg.timestamp_ns = sqlite3_column_int64(stmt, 1);
            const void* blob = sqlite3_column_blob(stmt, 2);
            int blob_size = sqlite3_column_bytes(stmt, 2);
            msg.data.assign(static_cast<const uint8_t*>(blob),
                            static_cast<const uint8_t*>(blob) + blob_size);
            fn(msg);
        }
        sqlite3_finalize(stmt);
    }

    // Count messages for a given topic id
    int64_t count_messages(int topic_id) const {
        std::string sql = "SELECT COUNT(*) FROM messages WHERE topic_id = " + std::to_string(topic_id);
        sqlite3_stmt* stmt = nullptr;
        sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
        int64_t count = 0;
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            count = sqlite3_column_int64(stmt, 0);
        }
        sqlite3_finalize(stmt);
        return count;
    }

   private:
    void load_topics() {
        const char* sql = "SELECT id, name, type, serialization_format FROM topics";
        sqlite3_stmt* stmt = nullptr;
        int rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr);
        if (rc != SQLITE_OK) {
            throw std::runtime_error("Cannot read topics table: " + std::string(sqlite3_errmsg(db_)));
        }
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            TopicInfo ti;
            ti.id = sqlite3_column_int(stmt, 0);
            ti.name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
            ti.type = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
            ti.serialization_format = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
            topics_[ti.id] = ti;
        }
        sqlite3_finalize(stmt);
    }

    sqlite3* db_ = nullptr;
    std::map<int, TopicInfo> topics_;
};

}  // namespace rosbag2

#endif  // INCLIO_ROSBAG2_READER_H
