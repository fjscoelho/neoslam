#ifndef MODE_GLOBALS_H_
#define MODE_GLOBALS_H_

#include <string>
#include <mutex>

/**
 * @brief Singleton para gerenciar o modo de operação global do sistema
 * 
 * Esta classe mantém o estado global do modo de operação (MAPPING/NAVIGATION)
 * para todo o sistema NeoSLAM, permitindo que diferentes nós consultem
 * e modifiquem o modo atual de forma thread-safe.
 * 
 * Modos disponíveis:
 *   - "mapping": Modo de mapeamento (criação de experiências)
 *   - "navigation": Modo de navegação (uso do mapa existente)
 */
class ModeGlobals {
public:
    /**
     * @brief Obtém a instância única do singleton
     * @return Referência para a instância
     */
    static ModeGlobals& getInstance() {
        static ModeGlobals instance;
        return instance;
    }

    /**
     * @brief Verifica se está em modo MAPPING
     * @return true se estiver em modo MAPPING, false caso contrário
     */
    bool isMappingMode() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return mode_ == "mapping";
    }

    /**
     * @brief Verifica se está em modo NAVIGATION
     * @return true se estiver em modo NAVIGATION, false caso contrário
     */
    bool isNavigationMode() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return mode_ == "navigation";
    }

    /**
     * @brief Define o modo de operação
     * @param mode "mapping" ou "navigation"
     */
    void setMode(const std::string& mode) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode == "mapping" || mode == "navigation") {
            mode_ = mode;
        }
    }

    /**
     * @brief Obtém o modo atual como string
     * @return "mapping" ou "navigation"
     */
    std::string getMode() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return mode_;
    }

    /**
     * @brief Alterna entre MAPPING e NAVIGATION
     * @return O novo modo ("mapping" ou "navigation")
     */
    std::string toggleMode() {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_ = (mode_ == "mapping") ? "navigation" : "mapping";
        return mode_;
    }

private:
    // Construtor privado (Singleton)
    ModeGlobals() : mode_("mapping") {}
    
    // Desabilita cópia e atribuição
    ModeGlobals(const ModeGlobals&) = delete;
    ModeGlobals& operator=(const ModeGlobals&) = delete;

    std::string mode_;
    mutable std::mutex mutex_;
};

#endif // MODE_GLOBALS_H_