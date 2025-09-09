
//////////////////////////////////////////////////////////////////////////////////
// Company: Gelisim University
// 
// Date: 2025
// Design Name: Autodrive Robot
// Module Name: i2c_master
// Project Name: Gelisim Robot
// Description: 
// 
//////////////////////////////////////////////////////////////////////////////////
module i2c_master(
        input wire clk,
        input wire rst_master,
        input wire [6:0] dev_addr_in,
        input wire [7:0] mem_addr_in,
        input wire [63:0] data_in,
        input wire [3:0] dataSize_in,
        input wire rw_bit_in,
        
        
        output wire [63:0] data_read_out,
        output wire stop_int,
        input wire SDA,
        output wire SCL,
        output wire SDA_oe_top,
        output wire SDA_reg_top
    );
    parameter IDLE       =0;
    parameter START      =1;
    parameter ADDRESS_RW =2;
    parameter ACK_in_ADDR=4;
    parameter ACK_in_DATA=5;
    parameter ACK_out    =6;
    parameter MEM_ADDR   =7;
    parameter STOP       =9;
    parameter SEND_DATA  =10;
    parameter READ_DATA  =11;
    
    reg read_en = 0;
    reg rw_bit;
    reg stop_signal = 0;
    reg start_signal = 0;
    reg sda_en = 0;
    reg scl_en = 0;
    reg sda_reg;
    reg scl_reg = 1;
    reg ack_wait = 0;
    reg [2:0]scl_sample;
    reg sda_outEn = 0;
    reg master_en;
    reg [7:0] dev_addr_rw;
    reg [7:0] mem_addr, scl_sample_count;
    reg [3:0] dataCounter;
    reg [3:0] counter;
    reg [3:0] dataSize;
    reg [3:0] state;
    reg [63:0] data_send;
    reg [63:0] data_read;
    reg [31:0] clk_counter;
    
    assign SCL=scl_reg;
    //assign SDA= sda_outEn ? sda_reg : 1'bz; //tristate
    assign data_read_out = data_read;
    assign SDA_oe_top = sda_outEn;
    assign SDA_reg_top = sda_reg;
    
    always@(posedge clk) begin
        if(master_en) begin
            clk_counter <= clk_counter + 1;
            if(clk_counter == 250) begin 
                scl_reg = 0;
            end
            if (clk_counter == 500) begin
                clk_counter <= 0;
                scl_reg = 1;
                scl_en = 1;
            end else scl_en = 0;
            if(clk_counter == 375) begin 
                sda_en = 1;
            end else sda_en = 0;
            
        end    
        scl_sample_count = scl_sample_count + 1;
        if (scl_sample_count == 63) begin
            scl_sample_count = 0;
            scl_sample = {scl_sample[1:0] , scl_reg};
        end
        
        if(rst_master == 1) begin
            state = START;
            read_en = 0;
            rw_bit = rw_bit_in;
            dev_addr_rw = {dev_addr_in, rw_bit};
            mem_addr = mem_addr_in;
            dataSize = dataSize_in;
            sda_en = 0; 
            scl_en = 0;
            stop_signal = 0;
            start_signal = 0;
            data_send = data_in;
            master_en = 1;
            clk_counter <= 0;
            scl_sample_count = 0;
            counter = 0;
            dataCounter = 0;
            data_read = 0;
        end
        
        if(start_signal && scl_sample[0] && scl_sample[1]) begin
            sda_outEn = 1;
            sda_reg = 0;
            state = ADDRESS_RW;
            counter = 0;
            start_signal = 0;
        end
        if(stop_signal && scl_sample[0] && scl_sample[1]) begin
            sda_outEn = 0; //sda is 1 because of pull ups
            state = IDLE;
            master_en = 0;
            stop_signal = 0;
        end
        
        if(sda_en) begin
            case (state)
                START: begin
                    start_signal = 1;
                    sda_outEn = 1;
                    sda_reg = 1;
                end
                
                ADDRESS_RW : begin
                    sda_outEn=1; //write op
                    sda_reg = dev_addr_rw[7 - counter]; 
                    counter = counter + 1;
                    if (counter == 9) begin
                        counter = 0;
                        state = ACK_in_ADDR;
                        ack_wait = 1;
                        sda_outEn = 0;
                    end
                end
        
                STOP : begin
                    stop_signal = 1;
                    sda_outEn = 1;
                    sda_reg = 0;
                end
                
                MEM_ADDR : begin
                    sda_outEn = 1;  //write op
                    sda_reg = mem_addr[7 - counter];
                    counter = counter + 1;
                    if (counter == 9) begin
                        counter=0;
                        state=ACK_in_DATA;
                        ack_wait = 1;
                        sda_outEn = 0;
                    end
                end
        
                SEND_DATA : begin
                    sda_outEn = 1; //write op
                    sda_reg = data_send[(dataCounter*8) + 7 - counter];
                    counter = counter + 1;
         
                    if (counter == 9) begin
                        dataCounter = dataCounter + 1;
                        counter = 0;
                        state=ACK_in_DATA;
                        ack_wait = 1;
                        sda_outEn = 0;
                    end
                end
                
                READ_DATA : begin
                    read_en = 1;
                    sda_outEn = 0;
                end
                
                ACK_out: begin
                    sda_outEn = 1; //write op
                    if (dataCounter < dataSize) begin 
                        sda_reg = 0; //ACK signal
                        state = READ_DATA;
                    end
                    else begin
                        sda_reg = 1; //NACK signal
                        state = STOP;
                    end
                end
            endcase
        end  
        
        
        if (scl_en) begin
            if(ack_wait) begin
                sda_outEn = 0;
                ack_wait = 0;
                if(!SDA) begin
                    if(state == ACK_in_ADDR) begin
                        if(!rw_bit) state = MEM_ADDR;
                        else state = READ_DATA;
                    end
                    if(state == ACK_in_DATA) begin
                        if(dataCounter < dataSize) begin
                            state = SEND_DATA;
                        end else state = STOP;
                    end
                end else begin
                    state = STOP;
                end
            end

            if(read_en) begin
                read_en = 0;
                sda_outEn=0; //read op
                data_read = {data_read[63:0], SDA};
                counter = counter + 1;

                if (counter==8) begin
                    counter = 0;
                    dataCounter = dataCounter + 1;
                    state=ACK_out;
                end
            end
        end  
    end 
    assign stop_int = ~master_en;
endmodule
