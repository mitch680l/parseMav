#pragma once
#include <map>
#include <string>
#include <memory>
#include <set>
#include <vector>
#include <deque>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <chrono>
#include <algorithm>
#include <cctype>

#include "Vehicle.h"
#include "Rover.h"
#include "Plane.h"
#include "Copter.h"
#include "Command.h"




class SpeechInterpreter {
public:
    /**
     * Constructor for single-vehicle interpreter
     * @param vehicleType Type of this vehicle ("rover", "plane", "copter")
     * @param callsign Unique identifier for this vehicle (e.g., "alpha", "rover 67", "delta 284")
     * @param fifoPath Path to the shared communication channel
     * @param actionsFile Path to file containing valid action keywords
     * @param actionTimeoutMs Timeout in milliseconds for action completion
     */
    explicit SpeechInterpreter(const std::string& vehicleType,
                              const std::string& callsign,
                              const std::string& fifoPath,
                              const std::string& actionsFile = ACTIONS_FILE,
                              int actionTimeoutMs = 3000)
        : m_vehicleType(normalize(vehicleType)), 
          m_fifoPath(fifoPath), 
          m_actionTimeoutMs(actionTimeoutMs) {
        
        // Parse and normalize multi-word callsign
        parseCallsign(callsign);
        
        // Create the specific vehicle instance
        if (!createVehicle()) {
            throw std::runtime_error("Failed to create vehicle of type: " + vehicleType);
        }
        
        // Load valid actions
        if (!loadActionsFromFile(actionsFile)) {
            std::cerr << "[" << getCallsignString() << "] Warning: Could not load actions from " 
                      << actionsFile << std::endl;
        }
        
        std::cout << "[" << getCallsignString() << "] Initialized " << m_vehicleType 
                  << " with " << m_validActions.size() << " valid actions" << std::endl;
        
        // Print configuration status
        printConfiguration();
    }

    /**
     * Destructor - Clean up resources
     */
    ~SpeechInterpreter() {
        stop();
        if (m_fifoFd >= 0) {
            close(m_fifoFd);
        }
    }

    /**
     * Start the interpreter
     */
    bool start() {
        if (!prepareFifo()) {
            return false;
        }
        
        m_running = true;
        std::cout << "[" << getCallsignString() << "] Started, listening for commands addressed to:" 
                  << std::endl;
        
        if (ENABLE_CALLSIGN_KEYWORDS) {
            std::cout << "  - Callsign: '" << getCallsignString() << "'" << std::endl;
        }
        if (ENABLE_VEHICLE_TYPE_KEYWORDS) {
            std::cout << "  - Vehicle type: '" << m_vehicleType << "'" << std::endl;
        }
        if (ENABLE_GLOBAL_KEYWORDS) {
            std::cout << "  - Global commands: 'all', 'global', 'everyone', 'everybody'" << std::endl;
        }
        if (ENABLE_TOGGLE_MODE) {
            std::cout << "  - Toggle mode: 'on/off' activation (currently " 
                      << (m_toggleActivated ? "ON" : "OFF") << ")" << std::endl;
        }
        if (ENABLE_NO_ID_MODE) {
            std::cout << "  - No-ID mode: Commands accepted without identification" << std::endl;
        }
        if (REQUIRE_OVER) {
            std::cout << "  - Commands require 'over' keyword to execute" << std::endl;
        }
        
        // Check if any addressing mode is available
        bool hasAddressingMode = ENABLE_CALLSIGN_KEYWORDS || ENABLE_VEHICLE_TYPE_KEYWORDS || 
                                ENABLE_GLOBAL_KEYWORDS || ENABLE_NO_ID_MODE || 
                                (ENABLE_TOGGLE_MODE && m_toggleActivated);
        
        if (!hasAddressingMode) {
            std::cout << "  - WARNING: No addressing modes enabled or activated!" << std::endl;
        }
        
        return true;
    }

    /**
     * Stop the interpreter
     */
    void stop() {
        m_running = false;
        if (m_inAction) {
            finishAction();
        }
    }

    /**
     * Main processing loop - call repeatedly or run in separate thread
     */
    bool processOnce() {
        if (!m_running) return false;
        
        std::string token = readToken();
        if (!token.empty()) {
            dispatchToken(token);
        }
        checkTimeout();
        
        return m_running;
    }

    /**
     * Blocking run loop
     */
    void run() {
        if (!start()) {
            std::cerr << "[" << getCallsignString() << "] Failed to start" << std::endl;
            return;
        }
        
        while (processOnce()) {
            usleep(1000); // 10ms sleep
        }
    }

    /**
     * Get vehicle information
     */
    struct VehicleInfo {
        std::string vehicleType;
        std::string callsign;
        bool isListening;
        bool inAction;
        std::string currentAction;
        std::vector<std::string> actionArgs;
        bool globalEnabled;
        bool vehicleTypeEnabled;
        bool callsignEnabled;
        bool toggleModeEnabled;
        bool noIdModeEnabled;
        bool toggleActivated;
        bool requireOver;
    };
    
    VehicleInfo getInfo() const {
        return {
            m_vehicleType,
            getCallsignString(),
            m_running,
            m_inAction,
            m_currentAction,
            m_actionArgs,
            ENABLE_GLOBAL_KEYWORDS,
            ENABLE_VEHICLE_TYPE_KEYWORDS,
            ENABLE_CALLSIGN_KEYWORDS,
            ENABLE_TOGGLE_MODE,
            ENABLE_NO_ID_MODE,
            m_toggleActivated,
            REQUIRE_OVER
        };
    }

    /**
     * Get valid actions for this vehicle
     */
    std::vector<std::string> getValidActions() const {
        return std::vector<std::string>(m_validActions.begin(), m_validActions.end());
    }

    /**
     * Manually set toggle state (for testing or external control)
     */
    void setToggleState(bool activated) {
        if (ENABLE_TOGGLE_MODE) {
            m_toggleActivated = activated;
            std::cout << "[" << getCallsignString() << "] Toggle mode manually set to: " 
                      << (activated ? "ON" : "OFF") << std::endl;
        }
    }

    /**
     * Get current toggle state
     */
    bool getToggleState() const {
        return m_toggleActivated;
    }

private:
    // Vehicle identity
    std::string m_vehicleType;  // "rover", "plane", "copter"
    std::vector<std::string> m_callsignTokens;  // Multi-word callsign as tokens
    std::unique_ptr<Vehicle> m_vehicle;
    
    // Configuration
    std::string m_fifoPath;
    int m_actionTimeoutMs;
    std::set<std::string> m_validActions;
    
    // Communication
    int m_fifoFd = -1;
    bool m_running = false;
    std::string m_readBuffer;
    std::deque<std::string> m_tokenQueue;
    
    // Command processing state
    enum class ListenState {
        WAITING_FOR_ADDRESS,    // Looking for callsign/vehicle type/global
        WAITING_FOR_ACTION,     // Address matched, looking for action
        COLLECTING_ARGS         // Action found, collecting arguments
    };
    
    ListenState m_state = ListenState::WAITING_FOR_ADDRESS;
    bool m_inAction = false;
    std::string m_currentAction;
    std::vector<std::string> m_actionArgs;
    std::vector<std::string> m_preActionArgs;  // Arguments collected before action keyword
    std::chrono::steady_clock::time_point m_actionStartTime;
    
    // Multi-word callsign matching state
    size_t m_callsignMatchIndex = 0;  // Current position in callsign matching
    
    // New toggle mode state
    bool m_toggleActivated = false;  // Whether toggle mode is currently active
    
    // Keywords
    static constexpr const char* END_KEYWORD = "over";
    static constexpr const char* CANCEL_KEYWORD = "cancel";
    static constexpr const char* TOGGLE_ON_KEYWORD = "on";
    static constexpr const char* TOGGLE_OFF_KEYWORD = "off";
    static const std::set<std::string> GLOBAL_KEYWORDS;

    /**
     * Parse multi-word callsign into normalized tokens
     */
    void parseCallsign(const std::string& callsign) {
        std::istringstream iss(callsign);
        std::string word;
        
        m_callsignTokens.clear();
        while (iss >> word) {
            std::string normalized = normalize(word);
            if (!normalized.empty()) {
                m_callsignTokens.push_back(normalized);
            }
        }
        
        if (m_callsignTokens.empty()) {
            throw std::runtime_error("Empty callsign provided");
        }
    }

    /**
     * Get callsign as a single string for display
     */
    std::string getCallsignString() const {
        std::string result;
        for (size_t i = 0; i < m_callsignTokens.size(); ++i) {
            if (i > 0) result += " ";
            result += m_callsignTokens[i];
        }
        return result;
    }

    /**
     * Print current configuration
     */
    void printConfiguration() const {
        std::cout << "[" << getCallsignString() << "] Configuration:" << std::endl;
        std::cout << "  - Callsign addressing: " << (ENABLE_CALLSIGN_KEYWORDS ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "  - Vehicle type addressing: " << (ENABLE_VEHICLE_TYPE_KEYWORDS ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "  - Global addressing: " << (ENABLE_GLOBAL_KEYWORDS ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "  - Toggle mode: " << (ENABLE_TOGGLE_MODE ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "  - No-ID mode: " << (ENABLE_NO_ID_MODE ? "ENABLED" : "DISABLED") << std::endl;
        std::cout << "  - Require 'over' keyword: " << (REQUIRE_OVER ? "ENABLED" : "DISABLED") << std::endl;
    }

    /**
     * Create the vehicle instance based on type
     */
    bool createVehicle() {
        try {
            if (m_vehicleType == "rover") {
                m_vehicle = std::make_unique<Rover>();
            } else if (m_vehicleType == "plane") {
                m_vehicle = std::make_unique<Plane>();
            } else if (m_vehicleType == "copter" || m_vehicleType == "helicopter") {
                m_vehicle = std::make_unique<Copter>();
            } else {
                std::cerr << "[" << getCallsignString() << "] Unknown vehicle type: " 
                          << m_vehicleType << std::endl;
                return false;
            }
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[" << getCallsignString() << "] Failed to create vehicle: " 
                      << e.what() << std::endl;
            return false;
        }
    }

    /**
     * Load valid actions from file
     */
    bool loadActionsFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "[" << getCallsignString() << "] Cannot open actions file: " 
                      << filename << std::endl;
            return false;
        }
        
        std::string line;
        int count = 0;
        while (std::getline(file, line)) {
            std::string action = normalize(line);
            if (!action.empty()) {
                m_validActions.insert(action);
                count++;
            }
        }
        
        std::cout << "[" << getCallsignString() << "] Loaded " << count 
                  << " actions from " << filename << std::endl;
        return true;
    }

    /**
     * Set up the named pipe
     */
    bool prepareFifo() {
        // Open existing FIFO for reading (non-blocking)
        m_fifoFd = open(m_fifoPath.c_str(), O_RDONLY | O_NONBLOCK);
        if (m_fifoFd < 0) {
            std::cerr << "[" << getCallsignString() << "] Failed to open FIFO " << m_fifoPath 
                      << ": " << strerror(errno) << std::endl;
            return false;
        }
        
        return true;
    }

    /**
     * Read and return the next available token
     */
    std::string readToken() {
        if (m_tokenQueue.empty()) {
            fillTokenQueue();
        }
        
        if (!m_tokenQueue.empty()) {
            std::string token = m_tokenQueue.front();
            m_tokenQueue.pop_front();
            return token;
        }
        
        return {};
    }

    /**
     * Read from FIFO and populate token queue
     */
    void fillTokenQueue() {
        char buffer[1024];
        ssize_t bytesRead = read(m_fifoFd, buffer, sizeof(buffer) - 1);
        
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';
            m_readBuffer.append(buffer);
            
            // Process complete words separated by whitespace (real-time processing)
            // Don't wait for complete lines - process tokens as they arrive
            std::istringstream iss(m_readBuffer);
            std::string word;
            std::string remaining;
            
            // Extract complete words
            while (iss >> word) {
                std::string normalized = normalize(word);
                if (!normalized.empty()) {
                    m_tokenQueue.push_back(normalized);
                }
            }
            
            // Keep any partial word at the end
            size_t lastSpace = m_readBuffer.find_last_of(" \t\n\r");
            if (lastSpace != std::string::npos) {
                remaining = m_readBuffer.substr(lastSpace + 1);
                m_readBuffer = remaining;
            } else if (!m_tokenQueue.empty()) {
                // If we extracted tokens, clear the buffer
                m_readBuffer.clear();
            }
            
        } else if (bytesRead < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "[" << getCallsignString() << "] FIFO read error: " 
                      << strerror(errno) << std::endl;
        }
    }

    /**
     * Split line into normalized tokens
     */
    void tokenizeLine(const std::string& line) {
        std::istringstream iss(line);
        std::string word;
        
        while (iss >> word) {
            std::string normalized = normalize(word);
            if (!normalized.empty()) {
                m_tokenQueue.push_back(normalized);
            }
        }
    }

    /**
     * Process a single token based on current state
     */
    void dispatchToken(const std::string& token) {
        // Handle end/cancel keywords in any state
        if (token == END_KEYWORD) {
            if (m_inAction) {
                finishAction();
            } else if (REQUIRE_OVER) {
                // "over" received but no action in progress - this could be end of rejected command
                std::cout << "[" << getCallsignString() << "] 'over' received (no active command)" << std::endl;
            }
            resetToWaitingState();
            return;
        }
        
        if (token == CANCEL_KEYWORD) {
            if (m_inAction) {
                std::cout << "[" << getCallsignString() << "] Action cancelled: " 
                          << m_currentAction << std::endl;
                m_inAction = false;
            }
            resetToWaitingState();
            return;
        }

        switch (m_state) {
            case ListenState::WAITING_FOR_ADDRESS:
                handleAddressToken(token);
                break;
                
            case ListenState::WAITING_FOR_ACTION:
                handleActionToken(token);
                break;
                
            case ListenState::COLLECTING_ARGS:
                handleArgumentToken(token);
                break;
        }
    }

    /**
     * Handle token when waiting for address (callsign/vehicle type/global)
     */
    void handleAddressToken(const std::string& token) {
        // If No-ID mode is enabled, try to handle as action directly
        if (ENABLE_NO_ID_MODE) {
            if (m_validActions.find(token) != m_validActions.end()) {
                std::cout << "[" << getCallsignString() << "] No-ID mode: Direct action" << std::endl;
                startAction(token);
                return;
            }
        }
        
        // If toggle mode is enabled and activated, try to handle as action directly
        if (ENABLE_TOGGLE_MODE && m_toggleActivated) {
            if (m_validActions.find(token) != m_validActions.end()) {
                std::cout << "[" << getCallsignString() << "] Toggle mode active: Direct action" << std::endl;
                startAction(token);
                return;
            }
        }
        
        // Try callsign matching first (if enabled)
        if (ENABLE_CALLSIGN_KEYWORDS && tryMatchCallsign(token)) {
            if (m_callsignMatchIndex >= m_callsignTokens.size()) {
                // Complete callsign match
                std::cout << "[" << getCallsignString() << "] Addressed by callsign" << std::endl;
                m_state = ListenState::WAITING_FOR_ACTION;
                m_preActionArgs.clear();
                m_callsignMatchIndex = 0;  // Reset for next time
                return;
            }
            // Partial match, continue waiting for more tokens
            return;
        }
        
        // Try vehicle type matching (if enabled)
        if (ENABLE_VEHICLE_TYPE_KEYWORDS && token == m_vehicleType) {
            std::cout << "[" << getCallsignString() << "] Addressed by vehicle type" << std::endl;
            m_state = ListenState::WAITING_FOR_ACTION;
            m_preActionArgs.clear();
            m_callsignMatchIndex = 0;  // Reset callsign matching
            return;
        }
        
        // Try global keyword matching (if enabled)
        if (ENABLE_GLOBAL_KEYWORDS && GLOBAL_KEYWORDS.find(token) != GLOBAL_KEYWORDS.end()) {
            std::cout << "[" << getCallsignString() << "] Addressed by global command: " << token << std::endl;
            m_state = ListenState::WAITING_FOR_ACTION;
            m_preActionArgs.clear();
            m_callsignMatchIndex = 0;  // Reset callsign matching
            return;
        }
        
        // No match - reset callsign matching and stay in WAITING_FOR_ADDRESS
        m_callsignMatchIndex = 0;
    }

    /**
     * Try to match token against callsign sequence
     * Returns true if token matches the current position in callsign
     */
    bool tryMatchCallsign(const std::string& token) {
        if (m_callsignMatchIndex < m_callsignTokens.size() && 
            token == m_callsignTokens[m_callsignMatchIndex]) {
            m_callsignMatchIndex++;
            return true;
        }
        
        // Check if this token could start a new callsign match
        if (m_callsignMatchIndex > 0 && token == m_callsignTokens[0]) {
            m_callsignMatchIndex = 1;
            return true;
        }
        
        // No match - reset
        m_callsignMatchIndex = 0;
        return false;
    }

    /**
     * Handle token when waiting for action
     */
    void handleActionToken(const std::string& token) {
        // Check for toggle mode commands first (if enabled)
        if (ENABLE_TOGGLE_MODE) {
            if (token == TOGGLE_ON_KEYWORD) {
                m_toggleActivated = true;
                std::cout << "[" << getCallsignString() << "] Toggle mode activated" << std::endl;
                resetToWaitingState();
                return;
            } else if (token == TOGGLE_OFF_KEYWORD) {
                m_toggleActivated = false;
                std::cout << "[" << getCallsignString() << "] Toggle mode deactivated" << std::endl;
                resetToWaitingState();
                return;
            }
        }
        
        // Check if this is a valid action
        if (m_validActions.find(token) != m_validActions.end()) {
            startAction(token);
        } else {
            // If not a valid action, check if it's another address
            // This handles cases like "rover 67 rover move" where addressing is repeated
            if (isAddressToken(token)) {
                handleAddressToken(token);
            } else {
                // Collect as pre-action argument
                m_preActionArgs.push_back(token);
                std::cout << "[" << getCallsignString() << "] Collected pre-action argument: " 
                          << token << std::endl;
            }
        }
    }

    /**
     * Check if token could be part of an address for this vehicle
     */
    bool isAddressToken(const std::string& token) const {
        // Check callsign tokens (if enabled)
        if (ENABLE_CALLSIGN_KEYWORDS) {
            for (const std::string& callsignToken : m_callsignTokens) {
                if (token == callsignToken) {
                    return true;
                }
            }
        }
        
        // Check vehicle type (if enabled)
        if (ENABLE_VEHICLE_TYPE_KEYWORDS && token == m_vehicleType) {
            return true;
        }
        
        // Check global keywords (if enabled)
        if (ENABLE_GLOBAL_KEYWORDS && GLOBAL_KEYWORDS.find(token) != GLOBAL_KEYWORDS.end()) {
            return true;
        }
        
        return false;
    }

    /**
     * Handle token when collecting arguments
     */
    void handleArgumentToken(const std::string& token) {
        // Check if this is a new action (interrupting current one)
        if (m_validActions.find(token) != m_validActions.end()) {
            std::cout << "[" << getCallsignString() << "] New action detected, finishing current action" 
                      << std::endl;
            finishAction();
            startAction(token);
        } else {
            // Add as argument
            m_actionArgs.push_back(token);
            std::cout << "[" << getCallsignString() << "] Added argument: " << token << std::endl;
        }
    }

    /**
     * Start a new action
     */
    void startAction(const std::string& action) {
        // Finish any current action
        if (m_inAction) {
            finishAction();
        }
        
        m_currentAction = action;
        
        // Combine pre-action arguments with post-action arguments
        // Pre-action args go first as they typically specify the type/mode of action
        m_actionArgs = m_preActionArgs;  // Start with pre-action arguments
        
        m_inAction = true;
        m_actionStartTime = std::chrono::steady_clock::now();
        m_state = ListenState::COLLECTING_ARGS;
        
        std::cout << "[" << getCallsignString() << "] Started action: " << action;
        if (!m_preActionArgs.empty()) {
            std::cout << " with prefix args: ";
            for (size_t i = 0; i < m_preActionArgs.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << "'" << m_preActionArgs[i] << "'";
            }
        }
        std::cout << std::endl;
    }

    /**
     * Reset to waiting for address state
     */
    void resetToWaitingState() {
        m_state = ListenState::WAITING_FOR_ADDRESS;
        m_preActionArgs.clear();  // Clear any collected pre-action arguments
        m_callsignMatchIndex = 0;  // Reset callsign matching
    }

    /**
     * Check for action timeout and handle REQUIRE_OVER logic
     */
    void checkTimeout() {
        if (!m_inAction) return;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - m_actionStartTime
        ).count();

        if (elapsed >= m_actionTimeoutMs) {
            if (REQUIRE_OVER) {
                // Auto-reject command if REQUIRE_OVER is enabled and timeout occurs
                std::cout << "[" << getCallsignString() << "] Command auto-rejected (timeout without 'over'): " 
                          << m_currentAction << std::endl;
                m_inAction = false;
                resetToWaitingState();
            } else {
                // Auto-complete command if REQUIRE_OVER is disabled
                std::cout << "[" << getCallsignString() << "] Action timeout, auto-completing: " 
                          << m_currentAction << std::endl;
                finishAction();
            }
        }
    }

    /**
     * Complete the current action
     */
    void finishAction() {
        if (!m_inAction || !m_vehicle) return;

        try {
            // Filter noise words from both prefix and suffix arguments
            std::vector<std::string> filteredPrefixArgs = filterNoiseWords(m_preActionArgs);
            
            // Suffix args are everything after the action keyword
            std::vector<std::string> suffixArgs(m_actionArgs.begin() + m_preActionArgs.size(), 
                                              m_actionArgs.end());
            std::vector<std::string> filteredSuffixArgs = filterNoiseWords(suffixArgs);
            
            // Create command with separate prefix and suffix arguments
            Command cmd(m_currentAction, filteredPrefixArgs, filteredSuffixArgs);
            
            std::cout << "[" << getCallsignString() << "] Executing: " << m_currentAction;
            if (!filteredPrefixArgs.empty()) {
                std::cout << " [prefix: ";
                for (size_t i = 0; i < filteredPrefixArgs.size(); ++i) {
                    if (i > 0) std::cout << ", ";
                    std::cout << filteredPrefixArgs[i];
                }
                std::cout << "]";
            }
            if (!filteredSuffixArgs.empty()) {
                std::cout << " [suffix: ";
                for (size_t i = 0; i < filteredSuffixArgs.size(); ++i) {
                    if (i > 0) std::cout << ", ";
                    std::cout << filteredSuffixArgs[i];
                }
                std::cout << "]";
            }
            std::cout << std::endl;
            
            m_vehicle->validateCommand(cmd);
            
        } catch (const std::exception& e) {
            std::cerr << "[" << getCallsignString() << "] Command execution failed: " 
                      << e.what() << std::endl;
        }

        m_inAction = false;
        resetToWaitingState();
    }

    /**
     * Filter common noise words from arguments
     */
    std::vector<std::string> filterNoiseWords(const std::vector<std::string>& args) const {
        static const std::set<std::string> noiseWords = {
            "the", "a", "an", "and", "or", "but", "in", "on", "at", "to", "for", "of", 
            "with", "by", "um", "uh", "like", "you", "know", "so", "well", "now", "then",
            "please", "can", "could", "would", "should"
        };
        
        std::vector<std::string> filtered;
        for (const std::string& arg : args) {
            if (noiseWords.find(arg) == noiseWords.end()) {
                filtered.push_back(arg);
            }
        }
        return filtered;
    }

    /**
     * Normalize a word
     */
    std::string normalize(const std::string& input) const {
        std::string output;
        output.reserve(input.size());
        
        for (char c : input) {
            if (std::isalnum(static_cast<unsigned char>(c))) {
                output.push_back(std::tolower(static_cast<unsigned char>(c)));
            }
        }
        
        return output;
    }
};

// Define global keywords
const std::set<std::string> SpeechInterpreter::GLOBAL_KEYWORDS = {
    "all", "global", "everyone", "everybody"
};