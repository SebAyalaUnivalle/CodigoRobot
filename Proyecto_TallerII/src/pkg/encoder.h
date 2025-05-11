#define ENCODER_H

class encoder{
    private:
    double DistanciaRecorrida;
    const double LongitudArco = 0.00816814; //La distancia en metros que recorre el robot cada vez que el encoder es activado.
    unsigned long ultimoTiempo;
    public:
    encoder();
    ~encoder();
    void init();
    void IncrementarDistancia();
    void ResetDistancia();
    double GetDistancia();
};