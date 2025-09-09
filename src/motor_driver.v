//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: motor_driver
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////
module motor_driver(
    input wire clk,
    
    input wire [2:0] motor_out,
    input wire [31:0] period,
    input wire [31:0] duty_cycle,
    
    output reg motor_driver_STBY,
    
    output wire pwm_out_rightmotor,
    output wire pwm_out_leftmotor,
    
    output reg direction_rightmotor_1, 
    output reg direction_rightmotor_2, 
    output reg direction_leftmotor_1, 
    output reg direction_leftmotor_2
    );
    
    //motor_out Constants
    parameter motor_forward     = 0;
    parameter motor_turn_left   = 1;
    parameter motor_turn_right  = 2;
    parameter motor_backward    = 3;
    parameter motor_stop        = 4;
    
    
    reg pwm_en_rightmotor = 0;
    reg pwm_en_leftmotor  = 0;
    
    

    pwm_generator pwm_motor_right (clk, pwm_en_rightmotor, period, duty_cycle, pwm_out_rightmotor);
    pwm_generator pwm_motor_left (clk, pwm_en_leftmotor, period, duty_cycle, pwm_out_leftmotor);

    always@(posedge clk) begin
        case(motor_out)
            motor_forward : begin
                motor_driver_STBY = 1;
                direction_rightmotor_1 = 1;
                direction_rightmotor_2 = 0;
                direction_leftmotor_1 = 1;
                direction_leftmotor_2 = 0;
                pwm_en_rightmotor = 1;
                pwm_en_leftmotor = 1;
            end
            
            motor_turn_left : begin
                motor_driver_STBY = 1;
                direction_rightmotor_1 = 1;   //ileri:
                direction_rightmotor_2 = 0;
                direction_leftmotor_1 = 0;    //geri
                direction_leftmotor_2 = 1;
                pwm_en_rightmotor = 1;
                pwm_en_leftmotor = 1;
            end
            
            motor_turn_right : begin
                motor_driver_STBY = 1;
                direction_rightmotor_1 = 0;   //geri
                direction_rightmotor_2 = 1;
                direction_leftmotor_1 = 1;    //ileri
                direction_leftmotor_2 = 0; 
                pwm_en_rightmotor = 1;
                pwm_en_leftmotor = 1;
            end
            
            motor_backward : begin
                motor_driver_STBY = 1;
                direction_rightmotor_1 = 0;   //geri
                direction_rightmotor_2 = 1;
                direction_leftmotor_1 = 0;      //geri
                direction_leftmotor_2 = 1;
                pwm_en_rightmotor = 1;
                pwm_en_leftmotor = 1;
            end
            
            motor_stop : begin
                motor_driver_STBY = 0;
                pwm_en_rightmotor = 0;
                pwm_en_leftmotor = 0;
            end
            
            default : begin
                motor_driver_STBY = 0;
                pwm_en_rightmotor = 0;
                pwm_en_leftmotor = 0;
            end
        endcase
    end

endmodule
