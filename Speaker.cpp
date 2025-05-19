#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

void speak(const std::vector<std::string>& words, const std::string& fifoPath) {
    std::ofstream pipe(fifoPath);
    if (!pipe.is_open()) {
        std::cerr << "Error: failed to open pipe for writing.\n";
        return;
    }

    for (const auto& word : words) {
        pipe << word << " ";
        pipe.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

    pipe << std::endl;
    pipe.close();
}

int main(int argc, char* argv[]) {
    std::string fifoPath = "/tmp/speech_pipe";

    if (argc < 2) {
        std::cerr << "Usage: ./speaker <words to send>\n";
        return 1;
    }

    std::vector<std::string> sentence(argv + 1, argv + argc);
    speak(sentence, fifoPath);

    return 0;
}
