int angleToPwm(float angleDeg) {
    float norm = (angleDeg + 90.0f) / 180.0f; // map [-90,90] to [0,1]
    return static_cast<int>(1000 + norm * 1000);
}