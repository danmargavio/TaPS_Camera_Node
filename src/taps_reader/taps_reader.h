//
// Created by kevin on 8/5/26.
// Updated for .taps v0x03 (AprilTag extras) — see common/taps_format.md
//

#ifndef TAPS_CAMERANODE_TAPS_READER_H
#define TAPS_CAMERANODE_TAPS_READER_H

#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

namespace fs = std::filesystem;

class TaPS_Reader {
public:
    enum class EncoderType : uint8_t {
        JPEG = 0x00,
        RAW = 0x01
    };

#pragma pack(push, 1)
    struct RawHeader {
        char signature[5];
        EncoderType encoding;
        uint64_t width;
        uint64_t height;
        double target_fps;
        uint32_t encoder_args_length;
    };

    struct FrameHeader {
        uint64_t frame_idx;
        int64_t nanoseconds;
        uint32_t frame_size;
    };
#pragma pack(pop)

    struct Header {
        EncoderType encoding;
        uint64_t width;
        uint64_t height;
        double target_fps;
        std::string encoder_args;
        uint64_t frame_count;
        // v0x03 extras (zeroed for v0x02 files)
        uint8_t version = 2;
        double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
        double tag_size_m = 0.0;
        std::string camera_alias;
        std::string tag_family;
    };

    // Parsed AprilTag record from a v0x03 per-frame metadata block.
    struct TagRecord {
        uint16_t tag_id;
        float decision_margin;
        uint8_t hamming;
        bool pose_valid;
        float translation[3];
        float rotation[9]; // row-major 3x3
        float reproj_error_rms_px;
        float tag_px_diag;
    };

    struct Frame {
        FrameHeader header;
        std::vector<uint8_t> data;
        std::vector<uint8_t> meta; // v0x03 tag metadata (empty for v0x02 / no detections)

        std::vector<TagRecord> parse_tags() const {
            std::vector<TagRecord> out;
            if (meta.empty()) return out;
            const size_t count = meta[0];
            if (meta.size() < 1 + count * 64) return out;
            out.reserve(count);
            for (size_t i = 0; i < count; ++i) {
                const uint8_t *p = meta.data() + 1 + i * 64;
                TagRecord t{};
                std::memcpy(&t.tag_id, p + 0, 2);
                std::memcpy(&t.decision_margin, p + 2, 4);
                t.hamming = p[6];
                t.pose_valid = p[7] != 0;
                std::memcpy(t.translation, p + 8, 12);
                std::memcpy(t.rotation, p + 20, 36);
                std::memcpy(&t.reproj_error_rms_px, p + 56, 4);
                std::memcpy(&t.tag_px_diag, p + 60, 4);
                out.push_back(t);
            }
            return out;
        }
    };

    explicit TaPS_Reader(const fs::path &path) {
        file_stream.open(path, std::ios::binary);
        if (!file_stream.is_open()) {
            throw std::runtime_error("Failed to open file: " + path.string());
        }

        header = read_header();
        first_frame_offset = file_stream.tellg();
    }

    Header read_header() {
        if (!file_stream.is_open()) {
            throw std::runtime_error("File is closed");
        }

        file_stream.seekg(0);

        RawHeader raw{};
        file_stream.read(reinterpret_cast<char *>(&raw), sizeof(RawHeader));
        if (!file_stream) {
            throw std::runtime_error("Failed to read raw header fields");
        }

        if (std::memcmp(raw.signature, "TaPS", 4) != 0) {
            throw std::runtime_error("File does not contain the TaPS header sequence");
        }
        if (raw.signature[4] != 0x02 && raw.signature[4] != 0x03) {
            throw std::runtime_error("Unsupported TaPS version (expected 0x02 or 0x03)");
        }

        Header parsed{};
        parsed.version = static_cast<uint8_t>(raw.signature[4]);
        parsed.encoding = raw.encoding;
        parsed.width = raw.width;
        parsed.height = raw.height;
        parsed.target_fps = raw.target_fps;

        parsed.encoder_args.resize(raw.encoder_args_length);
        if (raw.encoder_args_length > 0) {
            file_stream.read(&parsed.encoder_args[0], raw.encoder_args_length);
            if (!file_stream) {
                throw std::runtime_error("Failed to read encoder args string");
            }
        }

        file_stream.read(reinterpret_cast<char *>(&parsed.frame_count), sizeof(parsed.frame_count));
        if (!file_stream) {
            throw std::runtime_error("Failed to read frame count");
        }

        if (parsed.version == 3) {
            bool ok = true;
            ok = ok && file_stream.read(reinterpret_cast<char *>(&parsed.fx), sizeof(double));
            ok = ok && file_stream.read(reinterpret_cast<char *>(&parsed.fy), sizeof(double));
            ok = ok && file_stream.read(reinterpret_cast<char *>(&parsed.cx), sizeof(double));
            ok = ok && file_stream.read(reinterpret_cast<char *>(&parsed.cy), sizeof(double));
            ok = ok && file_stream.read(reinterpret_cast<char *>(&parsed.tag_size_m), sizeof(double));

            uint32_t alias_len = 0;
            ok = ok && file_stream.read(reinterpret_cast<char *>(&alias_len), sizeof(alias_len));
            if (ok && alias_len > 0) {
                parsed.camera_alias.resize(alias_len);
                ok = ok && file_stream.read(&parsed.camera_alias[0], alias_len);
            }

            uint32_t family_len = 0;
            ok = ok && file_stream.read(reinterpret_cast<char *>(&family_len), sizeof(family_len));
            if (ok && family_len > 0) {
                parsed.tag_family.resize(family_len);
                ok = ok && file_stream.read(&parsed.tag_family[0], family_len);
            }

            if (!ok) {
                throw std::runtime_error("Incomplete TaPS v3 header extras");
            }
        }

        return parsed;
    }

    bool read_next_frame(Frame &frame) {
        if (!file_stream.is_open()) {
            throw std::runtime_error("File is closed");
        }

        file_stream.read(reinterpret_cast<char *>(&frame.header), sizeof(FrameHeader));
        if (file_stream.gcount() == 0) {
            return false; // eof
        }
        if (!file_stream) {
            throw std::runtime_error("Incomplete frame header read");
        }

        uint32_t meta_size = 0;
        if (header.version == 3) {
            file_stream.read(reinterpret_cast<char *>(&meta_size), sizeof(meta_size));
            if (!file_stream) {
                throw std::runtime_error("Incomplete frame meta size read");
            }
        }

        frame.data.resize(frame.header.frame_size);
        if (frame.header.frame_size > 0) {
            file_stream.read(reinterpret_cast<char *>(frame.data.data()), frame.header.frame_size);
            if (!file_stream) {
                throw std::runtime_error("Incomplete frame payload read");
            }
        }

        frame.meta.clear();
        if (meta_size > 0) {
            frame.meta.resize(meta_size);
            file_stream.read(reinterpret_cast<char *>(frame.meta.data()), meta_size);
            if (!file_stream) {
                throw std::runtime_error("Incomplete frame meta read");
            }
        }

        return true;
    }

    void seek_to_first_frame() {
        if (!file_stream.is_open()) {
            throw std::runtime_error("File is closed");
        }
        file_stream.clear();
        file_stream.seekg(first_frame_offset, std::ios::beg);
    }

    void seek(const uint64_t target_frame_idx) {
        if (!file_stream.is_open()) {
            throw std::runtime_error("File is closed");
        }

        seek_to_first_frame();

        FrameHeader frame_hdr{};
        while (file_stream) {
            const std::streampos frame_start = file_stream.tellg();

            file_stream.read(reinterpret_cast<char *>(&frame_hdr), sizeof(FrameHeader));
            if (!file_stream) {
                throw std::runtime_error("Target frame index out of range or file corrupted");
            }

            uint32_t meta_size = 0;
            if (header.version == 3) {
                file_stream.read(reinterpret_cast<char *>(&meta_size), sizeof(meta_size));
                if (!file_stream) {
                    throw std::runtime_error("Target frame index out of range or file corrupted");
                }
            }

            if (frame_hdr.frame_idx == target_frame_idx) {
                file_stream.seekg(frame_start, std::ios::beg);
                return;
            }

            file_stream.seekg(static_cast<std::streamoff>(frame_hdr.frame_size + meta_size), std::ios::cur);
        }

        throw std::runtime_error("Frame index not found");
    }

    const Header &get_header() const { return header; }

    void close() {
        file_stream.close();
    }

private:
    std::ifstream file_stream;
    Header header{};
    std::streampos first_frame_offset{0};
};


#endif //TAPS_CAMERANODE_TAPS_READER_H
