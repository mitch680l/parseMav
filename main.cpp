#include "SpeechInterpreter.h"

int main() {
    SpeechInterpreter interpreter("/tmp/speech_pipe");
    interpreter.run();
    return 0;
}