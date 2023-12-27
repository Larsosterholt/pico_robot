#ifndef Encoder_h
#define Encoder_h

class Encoder {
public:
    static Encoder& getInstance(int encoderPinA = -1, int encoderPinB = -1);
    void initialize();
    //void attachInterrupt();
    //static void encoderISR();
    long getCount();
    void increment();
    void decrement();
    void updateSpeed();
    float getSpeed();
    void handleInterrupt();

private:
    Encoder(int encoderPinA, int encoderPinB);
    int _encoderPinA, _encoderPinB;
    volatile long _count;
    long _lastCount;
    unsigned long _lastTime;
    float _speed; // Speed in counts per second or other unit
};

#endif
