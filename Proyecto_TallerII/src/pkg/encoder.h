#define ENCODER_H

class encoder{
    private:
    double DistanciaRecorrida, LongitudArco;
    int ultimoTiempo;
    public:
    encoder();
    ~encoder();
    void init();
    void IncrementarDistancia();
    void ResetDistancia();
    double GetDistancia();
};