
struct IdleState
{

}

struct StreamingState
{

}

enum ControllerStates {
    Idle(IdleState),
    Streaming(StreamingState),
}

struct ImuData
{
    orientation: [f32; 4],
    angular_velocity: [f32; 3],
    linear_acceleration: [f32; 3],
}

enum Motors {
    LeftWheel,
    RightWheel,
}

struct MotorState
{
    motor: Motors,
    velocity: f32,
    position: f32,
}


struct DistanceSensorValue
{
    distance: f32,
}

enum SensorMessage {
    ImuData(ImuData),
    MotorState(MotorState),
    DistanceSensorValue(DistanceSensorValue),
}

struct MotorControlPWM
{
    motor: Motors,
    pwm: f32,
}

enum ControlMessage {
    MotorControlPWM(MotorControlPWM),
}