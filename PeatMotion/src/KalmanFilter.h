#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
public:
    KalmanFilter(float q, float r, float p, float initial_value) {
        this->q = q;
        this->r = r;
        this->p = p;
        this->x = initial_value;
    }

    float update(float measurement) {
        this->p = this->p + this->q;
        float k = this->p / (this->p + this->r);
        this->x = this->x + k * (measurement - this->x);
        this->p = (1 - k) * this->p;
        return this->x;
    }

private:
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float p; // Estimation error covariance
    float x; // Value
};

#endif // KALMAN_FILTER_H
