//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: mpu6050_gyro_driver
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////
module mpu6050_gyro_driver(
    input wire clk,
    input wire reset, 
    
    input wire gyro_en,
    
    output reg [15:0] yaw,
    
    input wire SDA,
    output wire SCL,
    output wire SDA_oe_top,
    output wire SDA_reg_top,
    
    input wire [7:0] DEVICE_ADDR,
    input wire [7:0] PWR_MGMT_1,
    input wire [7:0] SMPLRT_DIV,
    input wire [7:0] INT_ENABLE,
    input wire [7:0] GYRO_ZOUT,
    input wire [7:0] PWR_MGMT_1_data,
    input wire [31:0] SMPLRT_DIV_data,
    input wire [7:0] INT_ENABLE_data,
    input wire [31:0] timer1,
    input wire [31:0] timer2,
    input wire [15:0] timer3,
    input wire [15:0] timer4,
    input wire [31:0] timer5,
    input wire [15:0] dt,
    input wire [2:0] motor_out
    );
    
    parameter motor_forward     = 0;        
    //state parameters
    parameter step_time_counter =12;
    parameter initialize_step1  = 0;
    parameter initialize_step2  = 1;
    parameter initialize_step3  = 2;
    parameter initialize_step4  = 3;
    parameter initialize_step5  = 4;
    parameter initialize_step6  = 5;
    parameter get_yaw_data0     = 8;
    parameter get_yaw_data1     = 9;
    parameter get_yaw_data2     = 10;
    parameter get_yaw_data3     = 11;
    parameter get_yaw_data4     = 13;
    parameter get_yaw_data5     = 14;
    parameter get_yaw_data6     = 15;
    parameter get_yaw_data7     = 16;
    parameter get_yaw_data8     = 17;
    parameter get_yaw_data9     = 18;
    parameter yaw_alignment     = 19;

    reg [4:0] state;
    reg [4:0] prev_state;
    reg [31:0] time_counter;
    reg  [15:0] gyro_xtmp;
    reg  [15:0] gyro_x;
    reg  [15:0] yaw_dt;
    reg  [15:0] yaw_dt_reg;
    reg  [15:0] dt_reg;
    
    //I2C master değişkenleri
    reg rst_master;
    reg [6:0] dev_addr_in;
    reg [7:0] mem_addr_in;
    reg [63:0] data_in;
    reg [3:0] dataSize_in;
    reg rw_bit_in;
    wire i2c_stop_int;
    wire [63:0] data_read_out;
    
    //Divider değişkenleri
    reg div_reset;
    reg div_start_division;
    reg [31:0] div_dividend;    // Dividend input
    reg [31:0] div_divisor;     // Divisor input
    wire [31:0] div_quotient;   // Quotient output
    wire [31:0] div_remainder;  // Remainder output
    wire div_division_active;   // Declaration of division_active
  	wire div_division_done;
  	reg div_sign;
    
    
    
    i2c_master i2c_master (clk, rst_master, dev_addr_in, mem_addr_in,
                            data_in, dataSize_in, rw_bit_in, data_read_out, i2c_stop_int, SDA, SCL, 
                            SDA_oe_top, SDA_reg_top);
    
    Divider32bit divider (clk, div_reset, div_start_division, div_dividend, div_divisor,
                          div_quotient, div_remainder, div_division_active, div_division_done);
    
    always@(posedge clk or posedge reset) begin
        if(reset) begin
            state = initialize_step1;
            time_counter <= 0;
            yaw = 360;
        end else begin 
            if (gyro_en) begin
                case(state)
                    step_time_counter: begin
                        time_counter <= time_counter + 1;
                        if ((prev_state == initialize_step2) && (time_counter == timer1)) begin
                            time_counter <= 0;
                            state = initialize_step3;
                            prev_state = step_time_counter;
                        end
                        if ((prev_state == initialize_step4) && (time_counter == timer1)) begin
                            time_counter <= 0;
                            state = initialize_step5;
                            prev_state = step_time_counter;
                        end
                        if ((prev_state == initialize_step6) && (time_counter == timer2)) begin
                            time_counter <= 0;
                            state = get_yaw_data0;
                            prev_state = step_time_counter;
                        end
                        if ((prev_state == get_yaw_data1) && (time_counter == timer3)) begin //write op bekle
                            time_counter <= 0;
                            state = get_yaw_data2;
                            prev_state = step_time_counter;
                        end
                        if ((prev_state == get_yaw_data3) && (time_counter > timer4) && i2c_stop_int) begin //read op bekle
                            time_counter <= 0;
                            state = get_yaw_data4;
                            prev_state = step_time_counter;
                            gyro_xtmp = data_read_out [15:0];
                            gyro_x = data_read_out[15:0];
                            dt_reg = dt;
                        end
                        if ((prev_state == get_yaw_data4) && (time_counter == timer5)) begin 
                            time_counter <= 0;
                            state = get_yaw_data0;
                            prev_state = step_time_counter;
                        end
                    end
                    //PWR_MGMT_1 (0x6B) → 0x00
                    initialize_step1 : begin 
                        dev_addr_in = DEVICE_ADDR;
                        mem_addr_in = PWR_MGMT_1;
                        data_in = PWR_MGMT_1_data;
                        dataSize_in = 1;
                        rw_bit_in = 0;
                        rst_master = 1;
                        state = initialize_step2;
                        prev_state = initialize_step1;
                    end
                    initialize_step2 : begin
                        rst_master = 0;
                        state = step_time_counter; //1 saniye bekle
                        prev_state = initialize_step2;
                    end
                    //SMPLRT_DIV (0x19) → 0x07, CONFIG (0x1A) → 0x06, GYRO_CONFIG (0x1B) → 0x18, ACCEL_CONFIG (0x1C) → 0x18  
                    initialize_step3 : begin 
                        mem_addr_in = SMPLRT_DIV;
                        data_in = SMPLRT_DIV_data; 
                        dataSize_in = 4;
                        rw_bit_in = 0;
                        rst_master = 1;
                        state = initialize_step4;
                        prev_state = initialize_step3;
                    end
                    initialize_step4 : begin
                        rst_master = 0;
                        state = step_time_counter; //1 saniye bekle
                        prev_state = initialize_step4;
                    end
                    //INT_ENABLE (0x38) → 0x01
                    initialize_step5 : begin
                        mem_addr_in = INT_ENABLE;
                        data_in = INT_ENABLE_data;
                        dataSize_in = 1;
                        rw_bit_in = 0;
                        rst_master = 1;
                        state = initialize_step6;
                        prev_state = initialize_step5;
                    end
                    initialize_step6 : begin
                        rst_master = 0;
                        state = step_time_counter; //4 saniye bekle
                        prev_state = initialize_step6;
                    end
    
                    //İlk önce registeri işaretler
                    get_yaw_data0 : begin
                        mem_addr_in = GYRO_ZOUT;
                        data_in = 8'h00;
                        dataSize_in = 0;
                        rw_bit_in = 0;
                        rst_master = 1;
                        state = get_yaw_data1;
                        prev_state = get_yaw_data0;
                    end
                    get_yaw_data1 : begin
                        rst_master = 0;
                        state = step_time_counter; //write op bekle
                        prev_state = get_yaw_data1;
                    end
                    //İşaretlenen registerden okuma yap
                    get_yaw_data2 : begin
                        dataSize_in = 2;
                        rw_bit_in = 1;
                        rst_master = 1;
                        state = get_yaw_data3;
                        prev_state = get_yaw_data2;
                    end
                    
                    get_yaw_data3: begin
                        rst_master = 0;
                        state = step_time_counter; //read op bekle
                        prev_state = get_yaw_data3;
                    end
                    
                    get_yaw_data4 : begin
                        
                        if (gyro_x[15]) begin
                            yaw_dt = (~gyro_x) + 1;
                            div_sign = 1;
                            
                        end else begin
                            yaw_dt = gyro_x;
                            div_sign = 0;
                            
                        end
                        
                        prev_state = get_yaw_data4;
                        state = get_yaw_data5;
                        
                    end
                    
                    get_yaw_data5: begin
                        div_dividend = {16'b0, yaw_dt};
                        div_divisor = {16'b0, dt_reg};
                        div_reset = 1;
                        state = get_yaw_data6;
                    end
                    
                    get_yaw_data6: begin
                        div_reset = 0;
                        state = get_yaw_data7;
                    end
                    
                    get_yaw_data7: begin
                        state = get_yaw_data8;
                    end
                    
                    get_yaw_data8: begin
                        div_start_division = 1;
                        
                        if (!div_division_active && div_division_done) begin
                            yaw_dt_reg = div_quotient [15:0]; 
                            div_start_division = 0;
                            state = get_yaw_data9;
                        end else state = get_yaw_data8;
                        
                    end
                    
                    get_yaw_data9: begin
                        if (!div_sign) yaw = yaw - yaw_dt_reg;
                        else yaw = yaw + yaw_dt_reg;
                        prev_state = get_yaw_data4;
                        state = yaw_alignment;
                    end
                    
                    yaw_alignment: begin
                        if ((yaw < 90) && (motor_out == motor_forward))  begin
                            yaw = yaw + 360;
                        end else if ((yaw > 630) && (motor_out == motor_forward)) begin
                            yaw = yaw - 360;
                        end else yaw = yaw;
                        state = step_time_counter;
                    end
                endcase
            end
        end
    end 
endmodule
