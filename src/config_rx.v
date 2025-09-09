//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: config_rx
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////
module config_rx(
    input wire clk,
    input wire reset, 
    input wire config_transfer_en,
    output reg soft_reset_DONE,
    
    output reg [31:0] gyro_delay_counter_constant,
    output reg [7:0] clock_1MHz_counter_constant,
    output reg [31:0] period,
    output reg [31:0] duty_cycle,
    output reg [7:0] critical_distance,
    output reg signed [31:0] initial_position_x,
    output reg signed [31:0] initial_position_y,
    output reg signed [31:0] target_position_x,
    output reg signed [31:0] target_position_y,
    output reg [15:0] STOP_THRESHOLD,
    output reg [15:0] slow_clk_divider,
    output reg [15:0] turn_constant,
    output reg [31:0] PULSE_PER_CM,
    output reg [7:0] DEBOUNCE_TIME,
    output reg [31:0] ms_counter,
    output reg [7:0] DEVICE_ADDR,
    output reg [7:0] PWR_MGMT_1,
    output reg [7:0] SMPLRT_DIV,
    output reg [7:0] INT_ENABLE,
    output reg [7:0] GYRO_ZOUT,
    output reg [7:0] PWR_MGMT_1_data,
    output reg [31:0] SMPLRT_DIV_data,
    output reg [7:0] INT_ENABLE_data,
    output reg [31:0] timer1,
    output reg [31:0] timer2,
    output reg [15:0] timer3,
    output reg [15:0] timer4,
    output reg [31:0] timer5,
    output reg [15:0] dt,
    output reg [13:0] uart_CLKS_PER_BIT,
    
    output wire [7:0] ext_sensor_data,
    output wire [7:0] ext_mode_select,
    output wire [15:0] ext_yaw, 
    input wire UART_RX,
    output reg [7:0] ROBOT_TURN_COUNT
    );
    
    parameter IDLE = 0;
    parameter REG_ADDR = 1;
    parameter REG_DATA = 2;
    parameter RESET = 3;
    parameter MEM_WRITE = 4;

    reg [3:0] command;
    reg [7:0] soft_reset_SM;
    reg [31:0] buffer;
    reg soft_reset;
    wire config_transfer;
    
    reg mem_en;
    reg mem_we;
    reg [7:0] mem_reg_addr;
    reg [7:0] mem_reg_data_in;
    wire [7:0] mem_reg_data_out;
    
    
    reg [2:0] uart_RX_DV_counter;
    wire uart_RX_DV;
    wire [7:0] uart_RX_Byte;
    
    memory memory (clk, reset, mem_en, mem_we, mem_reg_addr, mem_reg_data_in, 
                    mem_reg_data_out, ext_yaw, ext_sensor_data, ext_mode_select);
    uart_rx uart_rx (clk, reset, UART_RX, uart_CLKS_PER_BIT, uart_RX_DV, uart_RX_Byte);
    
    assign config_transfer = soft_reset | config_transfer_en;
    
    always@(posedge clk or posedge reset) begin
        if (reset) begin
            command <= IDLE;
            mem_en <= 0;
            mem_we <= 0;
            soft_reset <= 0;
            uart_RX_DV_counter <= 0;
            soft_reset_SM <= 0;
            buffer = 0;
            soft_reset_DONE <= 0;
        end
        else begin
            if (uart_RX_DV) begin
                uart_RX_DV_counter <= uart_RX_DV_counter +1;
            end else uart_RX_DV_counter <= 0;
            
            if (uart_RX_DV_counter == 2'h2) begin
                case (command)
                    IDLE: begin
                        mem_en <= 0;
                        
                        if (uart_RX_Byte == 8'h18) begin //CANCEL
                            command <= IDLE;
                        end 
                        else if (uart_RX_Byte == 8'h24) begin //PROGRAM
                            command <= REG_ADDR;
                        end
                        else if (uart_RX_Byte == 8'h11) begin //SYSTEM RESET
                            command <= RESET;
                        end
                    end
                    
                    REG_ADDR: begin
                        if (uart_RX_Byte == 8'h18) begin //CANCEL
                            command <= IDLE;
                        end 
                        else begin
                            mem_reg_addr <= uart_RX_Byte;
                            command <= REG_DATA;
                        end
                    end
                    
                    REG_DATA: begin
                        if (uart_RX_Byte == 8'h18) begin //CANCEL
                            command <= IDLE;
                        end 
                        else begin
                            mem_reg_data_in <= uart_RX_Byte;
                            mem_we <= 1;
                            command <= MEM_WRITE;
                        end
                    end
                    
                    MEM_WRITE: begin
                        mem_en <= 1;
                        command <= IDLE; 
                    end
                    
                    RESET: begin
                        soft_reset <= 1;
                        command <= IDLE;
                    end
                endcase
            end
            
            if (config_transfer) begin
                case (soft_reset_SM)
                    0: begin
                        mem_reg_addr <= 0;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 1;
                    end
                    
                    1: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 1;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 2;
                    end
                    
                    2: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 2;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 3;
                    end
                    
                    3: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 3;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 4;
                    end
                    
                    4: begin
                        buffer [31:24] = mem_reg_data_out;
                        initial_position_x = buffer;
                        buffer = 0;
                        mem_reg_addr <= 4;
                        mem_we <= 0;
                        mem_en <= 1; 
                        soft_reset_SM <= 5;
                    end
                    
                    5: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 5;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 6;
                    end
                    
                    6: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 6;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 7;
                    end
                    
                    7: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 7;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 8;
                    end
                    
                    8: begin
                        buffer [31:24] = mem_reg_data_out;
                        initial_position_y = buffer;
                        buffer = 0;
                        mem_reg_addr <= 8;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 9;
                    end
                    
                    9: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 9;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 10;
                    end
                    
                    10: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 10;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 11;
                    end
                    
                    11: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 11;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 12;
                    end
                    
                    12: begin
                        buffer [31:24] = mem_reg_data_out;
                        target_position_x = buffer;
                        buffer = 0;
                        mem_reg_addr <= 12;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 13;
                    end
                    
                    13: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 13;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 14;
                    end
                    
                    14: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 14;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 15;
                    end
                    
                    15: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 15;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 16;
                    end
                    
                    16: begin
                        buffer [31:24] = mem_reg_data_out;
                        target_position_y = buffer;
                        buffer = 0;
                        mem_reg_addr <= 16;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 17;
                    end
                    
                    17: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 17;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 18;
                    end
                    
                    18: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 18;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 19;
                    end
                    
                    19: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 19;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 20;
                    end
                    
                    20: begin
                        buffer [31:24] = mem_reg_data_out;
                        gyro_delay_counter_constant = buffer;
                        buffer = 0;
                        mem_reg_addr <= 20;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 21;
                    end
                    
                    21: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 21;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 22;
                    end
                    
                    22: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 22;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 23;
                    end
                    
                    23: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 23;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 24;
                    end
                    
                    24: begin
                        buffer [31:24] = mem_reg_data_out;
                        period  = buffer;
                        buffer = 0;
                        mem_reg_addr <= 83;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 25;
                    end
                    
                    25: begin
                        buffer [7:0] = mem_reg_data_out;
                        critical_distance  = buffer [7:0];
                        buffer = 0;
                        mem_reg_addr <= 25;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 26;
                    end 
                    
                    26: begin
                        buffer [7:0] = mem_reg_data_out;
                        clock_1MHz_counter_constant = buffer [7:0];
                        buffer = 0;
                        mem_reg_addr <= 26;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 27;
                    end
                    
                    27: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 27;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 28;
                    end
                    
                    28: begin
                        buffer [15:8] = mem_reg_data_out;
                        STOP_THRESHOLD = buffer [15:0];
                        buffer = 0;
                        mem_reg_addr <= 28;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 29;
                    end
                    
                    29: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 29;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 30;
                    end
                    
                    30: begin
                        buffer [15:8] = mem_reg_data_out;
                        slow_clk_divider = buffer [15:0];
                        buffer = 0;
                        mem_reg_addr <= 30;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 31;
                    end
                    
                    31: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 31;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 32;
                    end
                    
                    32: begin
                        buffer [15:8] = mem_reg_data_out;
                        turn_constant = buffer [15:0];
                        buffer = 0;
                        mem_reg_addr <= 32;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 33;
                    end
                    
                    33: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 33;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 34;
                    end
                    
                    34: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 34;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 35;
                    end
                    
                    35: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 35;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 36;
                    end
                    
                    36: begin
                        buffer [31:24] = mem_reg_data_out;
                        PULSE_PER_CM = buffer;
                        buffer = 0;
                        mem_reg_addr <= 36;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 37;
                    end
                    
                    37: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 37;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 38;
                    end
                    
                    38: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 38;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 39;
                    end
                    
                    39: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 39;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 40;
                    end
                    
                    40: begin
                        buffer [31:24] = mem_reg_data_out;
                        DEBOUNCE_TIME = buffer [31:24];
                        buffer = 0;
                        mem_reg_addr <= 40;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 41;
                    end
                    
                    41: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 41;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 42;
                    end
                    
                    42: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 42;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 43;
                    end
                    
                    43: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 43;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 44;
                    end
                    
                    44: begin
                        buffer [31:24] = mem_reg_data_out;
                        duty_cycle = buffer;
                        buffer = 0;
                        mem_reg_addr <= 44;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 45;
                    end
                    
                    45: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 45;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 46;
                    end
                    
                    46: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 46;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 47;
                    end
                    
                    47: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 47;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 48;
                    end
                    
                    48: begin
                        buffer [31:24] = mem_reg_data_out;
                        ms_counter = buffer;
                        buffer = 0;
                        mem_reg_addr <= 48;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 49;
                    end
                    
                    49: begin
                        buffer [7:0] = mem_reg_data_out;
                        DEVICE_ADDR = buffer [7:0];
                        buffer = 0;
                        mem_reg_addr <= 49;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 50;
                    end
                    
                    50: begin
                        buffer [7:0] = mem_reg_data_out;
                        PWR_MGMT_1 = buffer [7:0];
                        buffer = 0;
                        mem_reg_addr <= 50;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 51;
                    end
                    
                    51: begin
                        buffer [7:0] = mem_reg_data_out;
                        SMPLRT_DIV = buffer [7:0];
                        buffer = 0;
                        mem_reg_addr <= 51;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 52;
                    end
                    
                    52: begin
                        buffer [7:0] = mem_reg_data_out;
                        INT_ENABLE = buffer [7:0];
                        buffer = 0;
                        mem_reg_addr <= 52;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 53;
                    end
                    
                    53: begin
                        buffer [7:0] = mem_reg_data_out;
                        GYRO_ZOUT = buffer [7:0];
                        buffer = 0;
                        mem_reg_addr <= 53;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 54;
                    end
                    
                    54: begin
                        buffer [7:0] = mem_reg_data_out;
                        PWR_MGMT_1_data = buffer [7:0];
                        buffer = 0;
                        mem_reg_addr <= 54;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 55;
                    end
                    
                    55: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 55;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 56;
                    end
                    
                    56: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 56;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 57;
                    end
                    
                    57: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 57;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 58;
                    end
                    
                    58: begin
                        buffer [31:24] = mem_reg_data_out;
                        SMPLRT_DIV_data = buffer;
                        buffer = 0;
                        mem_reg_addr <= 58;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 59;
                    end
                    
                    59: begin
                        buffer [7:0] = mem_reg_data_out;
                        INT_ENABLE_data = buffer;
                        buffer = 0;
                        mem_reg_addr <= 59;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 60;
                    end
                    
                    60: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 60;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 61;
                    end
                    
                    61: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 61;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 62;
                    end
                    
                    62: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 62;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 63;
                    end
                    
                    63: begin
                        buffer [31:24] = mem_reg_data_out;
                        timer1 = buffer;
                        buffer = 0;
                        mem_reg_addr <= 63;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 64;
                    end
                    
                    64: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 64;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 65;
                    end
                    
                    65: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 65;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 66;
                    end
                    
                    66: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 66;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 67;
                    end
                    
                    67: begin
                        buffer [31:24] = mem_reg_data_out;
                        timer2 = buffer;
                        buffer = 0;
                        mem_reg_addr <= 67;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 68;
                    end
                    
                    68: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 68;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 69;
                    end
                    
                    69: begin
                        buffer [15:8] = mem_reg_data_out;
                        timer3 = buffer [15:0];
                        buffer = 0;
                        mem_reg_addr <= 69;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 70;
                    end
                    
                    70: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 70;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 71;
                    end
                    
                    71: begin
                        buffer [15:8] = mem_reg_data_out;
                        timer4 = buffer;
                        buffer = 0;
                        mem_reg_addr <= 71;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 72;
                    end
                    
                    72: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 72;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 73;
                    end
                    
                    73: begin
                        buffer [15:8] = mem_reg_data_out;
                        mem_reg_addr <= 73;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 74;
                    end
                    
                    74: begin
                        buffer [23:16] = mem_reg_data_out;
                        mem_reg_addr <= 74;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 75;
                    end
                    
                    75: begin
                        buffer [31:24] = mem_reg_data_out;
                        timer5 = buffer;
                        buffer = 0;
                        mem_reg_addr <= 75;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 76;
                    end
                    
                    76: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 76;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 77;
                    end
                    
                    77: begin
                        buffer [15:8] = mem_reg_data_out;
                        dt = buffer;
                        buffer = 0;
                        mem_reg_addr <= 77;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 78;
                    end
                    
                    78: begin
                        buffer [7:0] = mem_reg_data_out;
                        mem_reg_addr <= 78;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 79;
                    end
                    
                    79: begin
                        buffer [15:8] = mem_reg_data_out;
                        uart_CLKS_PER_BIT = buffer[13:0];
                        buffer = 0;
                        mem_reg_addr <= 84;
                        mem_we <= 0;
                        mem_en <= 1;
                        soft_reset_SM <= 80;
                    end
                    
                    80: begin
                        buffer [7:0] = mem_reg_data_out;
                        ROBOT_TURN_COUNT = buffer[7:0];
                        buffer = 0;
                        mem_we <= 0;
                        mem_en <= 0;
                        soft_reset_SM <= 81; 
                    end
                    
                    81: begin
                        soft_reset <= 0;
                        soft_reset_DONE <= 1;
                        soft_reset_SM <= 0;
                    end
                endcase
            end else begin
                soft_reset_DONE <= 0;
            end
        end
    end
    
endmodule
