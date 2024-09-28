    #include <TMC2209.h>
    #include <AccelStepper.h>
    
    #define STEP_PIN 9
    #define DIR_PIN 10
    #define STEP_PIN_2 12
    #define DIR_PIN_2 11

    AccelStepper stepper1(1, STEP_PIN, DIR_PIN); // (Typeof driver: with 2 pins, STEP, DIR)
    AccelStepper stepper2(1, STEP_PIN_2, DIR_PIN_2);

    HardwareSerial & serial_stream = Serial1;

    const int RX_PIN = 18;
    const int TX_PIN = 17;
    const int32_t INIT_VELOCITY = 5000;
    const uint8_t RUN_CURRENT_PERCENT = 100;



    // Instantiate the two TMC2209
    TMC2209 stepper_driver_0;
    const TMC2209::SerialAddress SERIAL_ADDRESS_0 = TMC2209::SERIAL_ADDRESS_0;
    TMC2209 stepper_driver_1;
    const TMC2209::SerialAddress SERIAL_ADDRESS_1 = TMC2209::SERIAL_ADDRESS_1;
    const uint8_t REPLY_DELAY = 4;
    const long SERIAL_BAUD_RATE = 115200;

    const int is_someting_fucked_up_timer = 2000;

    // Position A, Position B, MaxSpeeed A, MaxSpeed B
    const int max_speed = 1000;
    const int max_accel = 1000;
    int move_matrix_accelstepper_a[7][6] = {
        {0, 0, max_speed, max_speed, max_accel/4, max_accel/4},
        {10000, -10000, max_speed, max_speed, max_accel/4, max_accel/4},
        {0, 0, max_speed, max_speed, max_accel/4, max_accel/4},
        {10000, -10000, max_speed, max_speed, max_accel/2, max_accel/2},
        {0, 0, max_speed, max_speed, max_accel/2, max_accel/2},
        {10000, -10000, max_speed, max_speed, max_accel, max_accel},
        {0, 0, max_speed, max_speed, max_accel, max_accel},
    };  
    
    const int max_speed_1 = 500;
    const int max_accel_1 = 500;
    int move_matrix_accelstepper_b[7][6] = {
        {8000, 8000, max_speed_1, max_speed_1, max_accel_1, max_accel_1},
        {1800, -800, max_speed_1, max_speed_1, max_accel_1, max_accel_1},
        {0, 0, max_speed_1, max_speed_1, max_accel_1, max_accel_1},
        {800, 800, max_speed_1, max_speed_1, max_accel_1, max_accel_1},
        {0, 0, max_speed_1, max_speed_1, max_accel_1, max_accel_1},
        {800, -800, max_speed_1, max_speed_1, max_accel_1, max_accel_1},
        {0, 0, max_speed_1, max_speed_1, max_accel_1, max_accel_1},
    };  
    
    void printMotorSettings(auto& settings_motor_a, auto& settings_motor_b){
        Serial.println("*************************");
        Serial.println("getSettings()");

        Serial.print("Setting name: is_communicating = ");
        Serial.print(settings_motor_a.is_communicating);
        Serial.print(" / ");
        Serial.println(settings_motor_b.is_communicating);

        Serial.print("Setting name: is_setup = ");
        Serial.print(settings_motor_a.is_setup);
        Serial.print(" / ");
        Serial.println(settings_motor_b.is_setup);

        Serial.print("Setting name: software_enabled = ");
        Serial.print(settings_motor_a.software_enabled);
        Serial.print(" / ");
        Serial.println(settings_motor_b.software_enabled);

        Serial.print("Setting name: microsteps_per_step = ");
        Serial.print(settings_motor_a.microsteps_per_step);
        Serial.print(" / ");
        Serial.println(settings_motor_b.microsteps_per_step);

        Serial.print("Setting name: inverse_motor_direction_enabled = ");
        Serial.print(settings_motor_a.inverse_motor_direction_enabled);
        Serial.print(" / ");
        Serial.println(settings_motor_b.inverse_motor_direction_enabled);

        Serial.print("Setting name: stealth_chop_enabled = ");
        Serial.print(settings_motor_a.stealth_chop_enabled);
        Serial.print(" / ");
        Serial.println(settings_motor_b.stealth_chop_enabled);

        Serial.print("Setting name: standstill_mode = ");
        Serial.print((settings_motor_a.standstill_mode == TMC2209::NORMAL) ? "normal" :
                    (settings_motor_a.standstill_mode == TMC2209::FREEWHEELING) ? "freewheeling" :
                    (settings_motor_a.standstill_mode == TMC2209::STRONG_BRAKING) ? "strong_braking" : "braking");
        Serial.print(" / ");
        Serial.println((settings_motor_b.standstill_mode == TMC2209::NORMAL) ? "normal" :
                    (settings_motor_b.standstill_mode == TMC2209::FREEWHEELING) ? "freewheeling" :
                    (settings_motor_b.standstill_mode == TMC2209::STRONG_BRAKING) ? "strong_braking" : "braking");

        Serial.print("Setting name: irun_percent = ");
        Serial.print(settings_motor_a.irun_percent);
        Serial.print(" / ");
        Serial.println(settings_motor_b.irun_percent);

        Serial.print("Setting name: irun_register_value = ");
        Serial.print(settings_motor_a.irun_register_value);
        Serial.print(" / ");
        Serial.println(settings_motor_b.irun_register_value);

        Serial.print("Setting name: ihold_percent = ");
        Serial.print(settings_motor_a.ihold_percent);
        Serial.print(" / ");
        Serial.println(settings_motor_b.ihold_percent);

        Serial.print("Setting name: ihold_register_value = ");
        Serial.print(settings_motor_a.ihold_register_value);
        Serial.print(" / ");
        Serial.println(settings_motor_b.ihold_register_value);

        Serial.print("Setting name: iholddelay_percent = ");
        Serial.print(settings_motor_a.iholddelay_percent);
        Serial.print(" / ");
        Serial.println(settings_motor_b.iholddelay_percent);

        Serial.print("Setting name: iholddelay_register_value = ");
        Serial.print(settings_motor_a.iholddelay_register_value);
        Serial.print(" / ");
        Serial.println(settings_motor_b.iholddelay_register_value);

        Serial.print("Setting name: automatic_current_scaling_enabled = ");
        Serial.print(settings_motor_a.automatic_current_scaling_enabled);
        Serial.print(" / ");
        Serial.println(settings_motor_b.automatic_current_scaling_enabled);

        Serial.print("Setting name: automatic_gradient_adaptation_enabled = ");
        Serial.print(settings_motor_a.automatic_gradient_adaptation_enabled);
        Serial.print(" / ");
        Serial.println(settings_motor_b.automatic_gradient_adaptation_enabled);

        Serial.print("Setting name: pwm_offset = ");
        Serial.print(settings_motor_a.pwm_offset);
        Serial.print(" / ");
        Serial.println(settings_motor_b.pwm_offset);

        Serial.print("Setting name: pwm_gradient = ");
        Serial.print(settings_motor_a.pwm_gradient);
        Serial.print(" / ");
        Serial.println(settings_motor_b.pwm_gradient);

        Serial.print("Setting name: cool_step_enabled = ");
        Serial.print(settings_motor_a.cool_step_enabled);
        Serial.print(" / ");
        Serial.println(settings_motor_b.cool_step_enabled);

        Serial.print("Setting name: analog_current_scaling_enabled = ");
        Serial.print(settings_motor_a.analog_current_scaling_enabled);
        Serial.print(" / ");
        Serial.println(settings_motor_b.analog_current_scaling_enabled);

        Serial.print("Setting name: internal_sense_resistors_enabled = ");
        Serial.print(settings_motor_a.internal_sense_resistors_enabled);
        Serial.print(" / ");
        Serial.println(settings_motor_b.internal_sense_resistors_enabled);

        Serial.println("*************************");
        Serial.println();
    }

    void printMotorStatus(auto& status_motor_a, auto& status_motor_b){
        Serial.println("*************************");
        Serial.println("getStatus()");

        Serial.print("Status name: over_temperature_warning = ");
        Serial.print(status_motor_a.over_temperature_warning);
        Serial.print(" / ");
        Serial.println(status_motor_b.over_temperature_warning);

        Serial.print("Status name: over_temperature_shutdown = ");
        Serial.print(status_motor_a.over_temperature_shutdown);
        Serial.print(" / ");
        Serial.println(status_motor_b.over_temperature_shutdown);

        Serial.print("Status name: short_to_ground_a = ");
        Serial.print(status_motor_a.short_to_ground_a);
        Serial.print(" / ");
        Serial.println(status_motor_b.short_to_ground_a);

        Serial.print("Status name: short_to_ground_b = ");
        Serial.print(status_motor_a.short_to_ground_b);
        Serial.print(" / ");
        Serial.println(status_motor_b.short_to_ground_b);

        Serial.print("Status name: low_side_short_a = ");
        Serial.print(status_motor_a.low_side_short_a);
        Serial.print(" / ");
        Serial.println(status_motor_b.low_side_short_a);

        Serial.print("Status name: low_side_short_b = ");
        Serial.print(status_motor_a.low_side_short_b);
        Serial.print(" / ");
        Serial.println(status_motor_b.low_side_short_b);

        Serial.print("Status name: open_load_a = ");
        Serial.print(status_motor_a.open_load_a);
        Serial.print(" / ");
        Serial.println(status_motor_b.open_load_a);

        Serial.print("Status name: open_load_b = ");
        Serial.print(status_motor_a.open_load_b);
        Serial.print(" / ");
        Serial.println(status_motor_b.open_load_b);

        Serial.print("Status name: over_temperature_120c = ");
        Serial.print(status_motor_a.over_temperature_120c);
        Serial.print(" / ");
        Serial.println(status_motor_b.over_temperature_120c);

        Serial.print("Status name: over_temperature_143c = ");
        Serial.print(status_motor_a.over_temperature_143c);
        Serial.print(" / ");
        Serial.println(status_motor_b.over_temperature_143c);

        Serial.print("Status name: over_temperature_150c = ");
        Serial.print(status_motor_a.over_temperature_150c);
        Serial.print(" / ");
        Serial.println(status_motor_b.over_temperature_150c);

        Serial.print("Status name: over_temperature_157c = ");
        Serial.print(status_motor_a.over_temperature_157c);
        Serial.print(" / ");
        Serial.println(status_motor_b.over_temperature_157c);

        Serial.print("Status name: current_scaling = ");
        Serial.print(status_motor_a.current_scaling);
        Serial.print(" / ");
        Serial.println(status_motor_b.current_scaling);

        Serial.print("Status name: stealth_chop_mode = ");
        Serial.print(status_motor_a.stealth_chop_mode);
        Serial.print(" / ");
        Serial.println(status_motor_b.stealth_chop_mode);

        Serial.print("Status name: standstill = ");
        Serial.print(status_motor_a.standstill);
        Serial.print(" / ");
        Serial.println(status_motor_b.standstill);

        Serial.println("*************************");
        Serial.println();
    }

    void moveMotors(int move_matrix[][6], int rows) {
        // Iterate through the move matrix
        for (int i = 0; i < rows; i++) {
            // Get target positions and max speeds from the matrix
            int target_position_1 = move_matrix[i][0];
            int target_position_2 = move_matrix[i][1];
            int max_speed_1 = move_matrix[i][2];
            int max_speed_2 = move_matrix[i][3];
            int accel_1 = move_matrix[i][4];
            int accel_2 = move_matrix[i][5];

            // Set speed and acceleration for both motors
            stepper1.setMaxSpeed(max_speed_1);
            stepper2.setMaxSpeed(max_speed_2);
            stepper1.setAcceleration(accel_1);
            stepper2.setAcceleration(accel_2);

            // Set target positions
            stepper1.moveTo(target_position_1);
            stepper2.moveTo(target_position_2);

            // Print current move details for debugging
            Serial.print("Moving to positions: ");
            Serial.print(target_position_1);
            Serial.print(", ");
            Serial.print(target_position_2);
            Serial.print(" with speeds: ");
            Serial.print(max_speed_1);
            Serial.print(", ");
            Serial.print(max_speed_2);
            Serial.print(" and accelerations: ");
            Serial.print(accel_1);
            Serial.print(", ");
            Serial.println(accel_2);

            // Non-blocking movement, wait for both motors to reach their positions
            while (stepper1.currentPosition() != target_position_1 || stepper2.currentPosition() != target_position_2) {
                stepper1.run();
                stepper2.run();
            }

            // Optional: Add a delay if the motors return to a neutral (0, 0) position
            // if (target_position_1 == 0 && target_position_2 == 0) {
            //     delay(10000);  // 10-second delay for pause at neutral position
            // }
        }
    }

    void setup(){
        Serial.begin(SERIAL_BAUD_RATE);

        stepper_driver_0.setup(serial_stream, SERIAL_BAUD_RATE, SERIAL_ADDRESS_0, RX_PIN, TX_PIN);
        stepper_driver_0.setReplyDelay(REPLY_DELAY);
        stepper_driver_1.setup(serial_stream, SERIAL_BAUD_RATE, SERIAL_ADDRESS_1, RX_PIN, TX_PIN);
        stepper_driver_1.setReplyDelay(REPLY_DELAY);

        stepper_driver_0.setRunCurrent(RUN_CURRENT_PERCENT);
        stepper_driver_0.setMicrostepsPerStep(8);
        stepper_driver_0.disableAutomaticCurrentScaling();
        stepper_driver_0.disableAutomaticGradientAdaptation();
        stepper_driver_0.disableInverseMotorDirection();
        stepper_driver_0.setPwmOffset(64);

        stepper_driver_1.setRunCurrent(RUN_CURRENT_PERCENT);
        stepper_driver_1.setMicrostepsPerStep(8);
        stepper_driver_1.disableAutomaticCurrentScaling();
        stepper_driver_1.disableAutomaticGradientAdaptation();
        stepper_driver_1.enableInverseMotorDirection();
        stepper_driver_1.setPwmOffset(64);
        
        stepper_driver_0.enable();
        stepper_driver_1.enable();

        stepper_driver_0.moveAtVelocity(0);
        stepper_driver_1.moveAtVelocity(0);

        stepper1.setMaxSpeed(1000);
        stepper1.setAcceleration(500);
        stepper1.setCurrentPosition(0);

        stepper2.setMaxSpeed(1000);
        stepper2.setAcceleration(500);
        stepper2.setCurrentPosition(0);

        // Iterate through move_matrix_accelstepper_a
        moveMotors(move_matrix_accelstepper_a, 7);
        delay(1000);
        //moveMotors(move_matrix_accelstepper_b, calculateRows(move_matrix_accelstepper_b));
        
        TMC2209::Settings settings_motor_a = stepper_driver_0.getSettings();
        TMC2209::Settings settings_motor_b = stepper_driver_1.getSettings();
        TMC2209::Status status_motor_a = stepper_driver_0.getStatus();
        TMC2209::Status status_motor_b = stepper_driver_1.getStatus();
        printMotorSettings(settings_motor_a, settings_motor_b);
        printMotorStatus(status_motor_a, status_motor_b);
    }

    void loop()
    {
    }