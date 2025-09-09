//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: robot_controller
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////
module robot_controller(
    input wire clk,
    input wire reset,
    
    input wire clock_1MHz, 
    input wire clock_1MHz_prev,
     
    input wire robot_controller_en,
    
    //Encoder Input Pins
    input wire A1, B1,            // Motor 1 için enkoder sinyalleri
    
    input wire [15:0] yaw,
    
    //SR04 Sensor Pins
    output wire trig_tx,
    input  wire echo_rx,
    input  wire [7:0] critical_distance,
    output wire [3:0] mux_sensor_select,
    
    //Motor PWM Parameters
    input wire [31:0] period,
    input wire [31:0] duty_cycle,
    
    //Motor Outputs
    output wire motor_driver_STBY,
    output wire pwm_out_rightmotor,
    output wire pwm_out_leftmotor,
    output wire direction_rightmotor_1, 
    output wire direction_rightmotor_2, 
    output wire direction_leftmotor_1, 
    output wire direction_leftmotor_2,
    
    // Initial and Target Positions
    input  wire signed [31:0] initial_position_x,
    input  wire signed [31:0] initial_position_y,
    input  wire signed [31:0] target_position_x,
    input  wire signed [31:0] target_position_y,
    
    // Current Position Output
    output wire  signed [31:0] current_x_o,
    output wire  signed [31:0] current_y_o,
    
    //Target Reached Flag
    output reg target_reached,
    
    output wire [7:0] robot_input_sensors,
    output wire [11:0] total_angle,
    output reg  [1:0] mode, 
    output wire [2:0] motor_out,
    
    input  wire [7:0]  ROBOT_TURN_COUNT,
    input  wire [15:0] STOP_THRESHOLD,
    input  wire [15:0] slow_clk_divider,
    input  wire [15:0] turn_constant,
    input  wire [31:0] PULSE_PER_CM,
    input  wire [7:0]  DEBOUNCE_TIME,
    input  wire [31:0] ms_counter, 
    
    input  wire [7:0]  ext_sensor_data,
    input  wire [7:0]  ext_mode_select
);

    // ---------------- İç durumlar ----------------
    reg  signed [31:0] current_x;
    reg  signed [31:0] current_y;
    reg  [7:0] state_angle_count;
    reg        autodrive_en;
    wire signed [31:0] forward_distance_x, forward_distance_y;
    wire [1:0] direction1;
    reg        get_data_En;
    wire [7:0] input_sensors_in;
    reg  [11:0] targeted_angle;
    reg  [11:0] total_angle_prev;
    wire [2:0] robot_motor_out;
    
    // obstacle_avoidance mode constants
    parameter control_forward   = 0;
    parameter control_right     = 1;
    parameter control_left      = 2;
    parameter control_back      = 3;

    // State Machine States
    reg [3:0] state;
    parameter STATE_X               = 4'b000; // X ekseninde ilerleme
    parameter STATE_Y               = 4'b001; // Y ekseninde ilerleme
    parameter STATE_ANGLE           = 4'b010; // Hedef için açı ayarlama (sensör toplama)
    parameter STATE_DONE            = 4'b011; // Hedefe ulaşıldı
    parameter STATE_Y_TURN          = 4'b1000; 
    parameter MEASURING             = 4'b100;
    parameter STATE_Y_TURN_COMPLETE = 4'b101;
    parameter STATE_X_TURN_COMPLETE = 4'b110;
    parameter IDLE                  = 4'b111;

    assign robot_input_sensors = ext_mode_select[0] ? ext_sensor_data : input_sensors_in; 
    assign robot_motor_out = ext_mode_select[4] ? ext_mode_select[7:5] : motor_out;
    
    assign current_x_o = current_x;
    assign current_y_o = current_y;
    
    reg signed [31:0] c1m_d1x, c1m_d2x,c1m_d1y, c1m_d2y;
    reg  [15:0] c1m_d2t, c1m_d1t;
    
    // ---------------- Alt modüller ----------------
    autodrive obstacle_avoidance (
        .clk(clk), .reset(reset), .autodrive_en(autodrive_en), .input_sensors_in(robot_input_sensors), 
        .yaw_in(yaw), .mode(mode), .total_angle(total_angle), .motor_out(motor_out), 
        .slow_clk_divider(slow_clk_divider), .turn_constant(turn_constant), .targeted_angle(targeted_angle)
    );
 
    odometer odometer (
        .clk(clk), .reset(reset),  .clock_1MHz(clock_1MHz), .clock_1MHz_prev(clock_1MHz_prev),
        .motor_out(robot_motor_out), .total_angle(c1m_d2t), .A1(A1), .B1(B1), .direction1(direction1), 
        .forward_distance_x(forward_distance_x), .forward_distance_y(forward_distance_y),
        .PULSE_PER_CM(PULSE_PER_CM), .DEBOUNCE_TIME(DEBOUNCE_TIME)
    );

    motor_driver motor_driver (
        clk, robot_motor_out, period, duty_cycle, motor_driver_STBY,  
        pwm_out_rightmotor, pwm_out_leftmotor,
        direction_rightmotor_1, direction_rightmotor_2,
        direction_leftmotor_1, direction_leftmotor_2
    ); 
    
    get_8sensor_data sensor (
        .clk(clk), .reset(reset), 
        .clock_1MHz(clock_1MHz), .clock_1MHz_prev(clock_1MHz_prev),
        .critical_distance(critical_distance),
        .get_data_En(get_data_En),
        .trig_tx(trig_tx), .echo_rx(echo_rx),
        .mux_sensor_select(mux_sensor_select), 
        .input_sensors_in(input_sensors_in),
        .ms_counter(ms_counter)
    );
    
    wire signed [31:0] stop_th_s  = $signed({{16{1'b0}}, STOP_THRESHOLD});    
    wire x_in_window; 
    assign x_in_window = (current_x >= (target_position_x - stop_th_s)) && (current_x <= (target_position_x + stop_th_s));
    wire y_in_window;
    assign y_in_window = (current_y >= (target_position_y - stop_th_s)) && (current_y <= (target_position_y + stop_th_s));

    wire total_angle_in_window;
//    assign total_angle_in_window = ((mode == control_left) && ( c1m_d2t == (total_angle_prev - 90))) || ((mode == control_right) && (c1m_d2t == (total_angle_prev + 90)));
    assign total_angle_in_window = ((mode == control_left) && ( c1m_d2t == (total_angle_prev - 90))) ? 1 : 
                                        ((mode == control_right) && (c1m_d2t == (total_angle_prev + 90))) ? 1 : 0;
      
    always @(posedge clk or posedge reset) begin
        if (reset) begin 
            c1m_d1t <= 1'b0;
            c1m_d2t<= 1'b0;
        end else begin
            c1m_d1t <= total_angle; // asenkron giriş
            c1m_d2t <= c1m_d1t;     // senkron kopya
        end
    end
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            c1m_d1x <= 1'b0;
            c1m_d2x<= 1'b0;
        end else begin
            c1m_d1x <= forward_distance_x; // asenkron giriş
            c1m_d2x <= c1m_d1x;     // senkron kopya
        end
    end
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            c1m_d1y <= 1'b0;
            c1m_d2y <= 1'b0;
        end else begin
            c1m_d1y <= forward_distance_y; // asenkron giriş
            c1m_d2y <= c1m_d1y;     // senkron kopya
        end
    end
    

// --- Ana FSM
always @(posedge clk or posedge reset) begin   
    if (reset) begin
        state             <= IDLE;
        state_angle_count <= 8'd0;
        target_reached    <= 1'b0;
        mode              <= control_forward;
        targeted_angle    <= 12'd360;
        total_angle_prev  <= 12'd360;
        get_data_En       <= 1'b0;
        autodrive_en      <= 1'b0;
        current_x         <= initial_position_x;
        current_y         <= initial_position_y;
    end else if (robot_controller_en) begin
            current_x <= initial_position_x + c1m_d2x;
            current_y <= initial_position_y + c1m_d2y;
         
            case (state)
                IDLE: begin
                    get_data_En    <= 1;
                    autodrive_en   <= 1;
                    mode           <= control_forward;
                    target_reached <= 0;
                    state <= STATE_Y;
                  
                end
                // --- Başlangıç: sensör toplama → Y eksenine git
                STATE_ANGLE: begin
                    if (state_angle_count < ROBOT_TURN_COUNT) begin
                        get_data_En  <= 1'b1;
                        autodrive_en <= 1'b1;
                        mode         <= control_forward;
                        state        <= STATE_Y_TURN;   // <<< önce Y
                    end else begin
                        get_data_En      <= 1'b0;
                        autodrive_en     <= 1'b0;
                        total_angle_prev <= c1m_d2t;
                        mode             <= control_forward;
                        state            <= STATE_ANGLE; 
                    end 
                end
                
                STATE_Y_TURN: begin
                    if ((current_y < target_position_y) && (targeted_angle == 450)) begin
                            targeted_angle <= targeted_angle - 90;    // kuzeye dön
                            mode           <= control_left;
                    end else if ((current_y < target_position_y) && (targeted_angle == 270)) begin
                            targeted_angle <= targeted_angle + 90;    // kuzeye dön
                            mode           <= control_right;
                    end else if ((current_y > target_position_y) && (targeted_angle == 450))begin
                        targeted_angle <= targeted_angle + 90;   // güneye dön
                        mode           <= control_right;
                    end else if ((current_y > target_position_y) && (targeted_angle == 270))begin
                        targeted_angle <= targeted_angle - 90;   // güneye dön
                        mode           <= control_left;
                    end
                    total_angle_prev <= c1m_d2t;
                    state <= STATE_Y_TURN_COMPLETE;
                end
                
                STATE_Y_TURN_COMPLETE: begin
                   
                   if (total_angle_in_window) begin
                        mode  <= control_forward;
                        state <= STATE_Y;
                    end
                end

                // --- Y boyunca ilerle, pencereye girince X yönüne dön
                STATE_Y: begin
                    get_data_En    <= 1;
                    autodrive_en   <= 1;
                    mode <= control_forward;
                    if (y_in_window) begin
                        // hedef X yönünü belirle                       
                        if ((current_x < target_position_x) && (targeted_angle == 360)) begin
                            targeted_angle <= targeted_angle + 90;    // doğuya dön
                            mode           <= control_right;
                        end else if ((current_x < target_position_x) && (targeted_angle == 180)) begin
                            targeted_angle <= targeted_angle - 90;    // doğuya dön
                            mode           <= control_left;
                        end else if ((current_x > target_position_x) && (targeted_angle == 360))begin
                            targeted_angle <= targeted_angle - 90;   // güneye dön
                            mode           <= control_left;
                        end else if ((current_x > target_position_x) && (targeted_angle == 180))begin
                            targeted_angle <= targeted_angle + 90;   // güneye dön
                            mode           <= control_right;
                        end
                        total_angle_prev <= c1m_d2t; 
                        state <= STATE_X_TURN_COMPLETE;
                    end
                end

                // --- X yönüne dön ve hedef açıyı yakalayınca X hareketine geç
                STATE_X_TURN_COMPLETE: begin
                   
                   if (total_angle_in_window) begin
                        mode  <= control_forward;
                        state <= STATE_X;
                    end
                end

                // --- X boyunca ilerle, pencereye girince DONE
                STATE_X: begin
                    get_data_En    <= 1;
                    autodrive_en   <= 1;
                    mode <= control_forward;
                    if (x_in_window) begin
                        state <= STATE_DONE;
                    end
                end

                // --- Hedefe ulaşıldı mı?
                STATE_DONE: begin
                    if (x_in_window && y_in_window) begin
                        target_reached <= 1'b1;
                        get_data_En    <= 1'b0;
                        autodrive_en   <= 1'b0;
                        mode           <= control_forward;
                        state <= STATE_DONE;
                    end else begin
                        state_angle_count <= state_angle_count + 1;
                        get_data_En    <= 1'b1;
                        autodrive_en   <= 1'b1;
                        target_reached <= 1'b0;
                        state          <= STATE_ANGLE;
                    end
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        
            if (targeted_angle == 90) begin 
                targeted_angle <= 450;
            end
            if (targeted_angle == 540) begin 
                targeted_angle <= 180;
            end
        end else begin
            get_data_En    <= 1'b0;
            autodrive_en   <= 1'b0;
        end
    
end
endmodule