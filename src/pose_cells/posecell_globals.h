#ifndef POSECELL_GLOBALS_H_
#define POSECELL_GLOBALS_H_

#include <string>
#include <mutex>

class PosecellGlobals {
public:
    static PosecellGlobals& getInstance() {
        static PosecellGlobals instance;
        return instance;
    }

    // Getter com thread-safety
    bool isMappingMode() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return mode_ == "mapping";
    }

    bool isNavigationMode() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return mode_ == "navigation";
    }

    void setMode(const std::string& mode) {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_ = mode;
    }

    std::string getMode() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return mode_;
    }

private:
    PosecellGlobals() : mode_("mapping") {}
    PosecellGlobals(const PosecellGlobals&) = delete;
    PosecellGlobals& operator=(const PosecellGlobals&) = delete;

    std::string mode_;
    mutable std::mutex mutex_;
};

#endif // POSECELL_GLOBALS_H_