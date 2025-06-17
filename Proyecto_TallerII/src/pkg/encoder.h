#define ENCODER_H

class encoder{
    private:
        double DistanciaRecorrida;
        const double LongitudArco = 0.0040841; //La distancia en metros que recorre el robot cada vez que el encoder es activado.
        unsigned int ultimoTiempo;
    
    public:
        encoder();
        ~encoder();
        void init();
        void IncrementarDistancia();
        void ResetDistancia();
        double GetDistancia();
};