#ifndef PID_HPP
#define PID_HPP

template <typename Float>
class PositionalPID {
    Float Kp;
    Float Ki;
    Float Kd;
    Float I;
    Float setpoint;
    Float previous_error;

public:
    PositionalPID() = delete;
    PositionalPID(const PositionalPID&) = delete;
    PositionalPID(PositionalPID&&) = delete;

    PositionalPID& operator=(const PositionalPID&) = delete;
    PositionalPID& operator=(PositionalPID&&) = delete;

    ~PositionalPID() = default;

    PositionalPID(Float Kp_, Float Ki_, Float Kd_)
            : Kp(Kp_), Ki(Ki_), Kd(Kd_), I(0), setpoint(0), previous_error(0) {
        // nothing to do
    }

    void set_constants(Float Kp_, Float Ki_, Float Kd_) {
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
    }

    void set_setpoint(Float setpoint_) {
        setpoint = setpoint_;
    }

    Float get_setpoint() const {
        return setpoint;
    }

    template <typename Diff = Float(*)(Float, Float)>
    Float compute(Float measured, Float timestep, Float* P_ = nullptr, Float* I_ = nullptr,
            Float* D_ = nullptr, Diff diff = [] (Float a, Float b) -> Float { return a - b; }) {
        Float error = diff(measured, setpoint);

        // update PID control variable
        const Float P = Kp * error;
        I += error * timestep;
        const Float D = Kd * (error - previous_error) / timestep;
        previous_error = error;

        const Float response_PD = P + D;
        const Float response_I = Ki * I;
        Float response = response_PD + response_I;

        // backcalculate to solve I-term windup
        if (response > 1.f) {
            if (response_PD > 1.f) {
                I = 0.f;
            } else {
                I = (1.f - response_PD) / Ki;
            }
            response = 1.f;
        } else if (response < -1.f) {
            if (response_PD < -1.f) {
                I = 0.f;
            } else {
                I = (-1.f - response_PD) / Ki;
            }
            response = -1.f;
        }

        // output debug values if desired
        if (P_) {
            *P_ = P;
        }
        if (I_) {
            *I_ = response_I;
        }
        if (D_) {
            *D_ = D;
        }

        return response;
    }
};

#endif /* PID_HPP */