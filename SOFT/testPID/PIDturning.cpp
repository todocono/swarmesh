// Integrate the code below into the int turning() in testPID.ino

int turning(int deg, char dir)
{
    int num1 = (dir == 'L') ? 1 : 0;
    int num2 = !num1;
    int tick = _motor1.get_tick();
    if (tick < _PULSE * deg)
    {
        // calculate the error in angle
        int error = _PULSE * deg - tick;
        int gap = error * 0.1;
        _motor1.motor_move(100 + gap, num1);
        _motor2.motor_move(100 + gap, num2);
        return 0;
    }
    else
    {
        // calculate the current angle
        for (int i = 0; i < 2; i++)
        {
            _motors[i].motor_stop();
            _prev_tick[i] = 0;
        }
    }
}