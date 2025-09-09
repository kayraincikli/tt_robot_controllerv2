// Set Parameter CLKS_PER_BIT as follows:
// CLKS_PER_BIT = (Frequency of i_Clock)/(Frequency of UART)
// Example: 10 MHz Clock, 115200 baud UART
// (10000000)/(115200) = 87

module uart_rx (
  input       i_Clock,
  input       i_Reset,
  input       i_RX_Serial,
  input [13:0] i_CLKS_PER_BIT,
  output      o_RX_DV,
  output [7:0] o_RX_Byte
);

  // State Definitions
  parameter IDLE         = 3'b000;
  parameter RX_START_BIT = 3'b001;
  parameter RX_DATA_BITS = 3'b010;
  parameter RX_STOP_BIT  = 3'b011;
  parameter CLEANUP      = 3'b100;

  reg [13:0] r_Clock_Count = 0;
  reg [2:0] r_Bit_Index   = 0;
  reg [7:0] r_RX_Byte     = 0;
  reg       r_RX_DV       = 0;
  reg [2:0] r_SM_Main     = 0;
  reg [13:0] r_Latched_CLKS_PER_BIT = 0; 
  reg [1:0] r_DV_Extend_Count = 0; 

  always @(posedge i_Clock or posedge i_Reset) begin
    if (i_Reset) begin
      r_SM_Main              <= IDLE;
      r_Clock_Count          <= 0;
      r_Bit_Index            <= 0;
      r_RX_Byte              <= 0;
      r_RX_DV                <= 0;
      //r_Latched_CLKS_PER_BIT <= i_CLKS_PER_BIT;
      r_DV_Extend_Count      <= 0;
    end else begin
      r_Latched_CLKS_PER_BIT <= i_CLKS_PER_BIT;
      case (r_SM_Main)
        IDLE: begin
          r_RX_DV       <= 1'b0;
          r_Clock_Count <= 0;
          r_Bit_Index   <= 0;

          if (i_RX_Serial == 1'b0) begin // Start Bit Detected
            r_SM_Main <= RX_START_BIT;
          end else begin
            r_SM_Main <= IDLE;
          end
        end

        RX_START_BIT: begin
          if (r_Clock_Count == (r_Latched_CLKS_PER_BIT - 1) / 2) begin
            if (i_RX_Serial == 1'b0) begin
              r_Clock_Count <= 0;
              r_SM_Main     <= RX_DATA_BITS;
            end else begin
              r_SM_Main <= IDLE;
            end
          end else begin
            r_Clock_Count <= r_Clock_Count + 1;
            r_SM_Main     <= RX_START_BIT;
          end
        end

        RX_DATA_BITS: begin
          if (r_Clock_Count < r_Latched_CLKS_PER_BIT - 1) begin
            r_Clock_Count <= r_Clock_Count + 1;
            r_SM_Main     <= RX_DATA_BITS;
          end else begin
            r_Clock_Count          <= 0;
            r_RX_Byte[r_Bit_Index] <= i_RX_Serial;
            if (r_Bit_Index < 7) begin
              r_Bit_Index <= r_Bit_Index + 1;
              r_SM_Main   <= RX_DATA_BITS;
            end else begin
              r_Bit_Index <= 0;
              r_SM_Main   <= RX_STOP_BIT;
            end
          end
        end

        RX_STOP_BIT: begin
          if (r_Clock_Count < r_Latched_CLKS_PER_BIT - 1) begin
            r_Clock_Count <= r_Clock_Count + 1;
            r_SM_Main     <= RX_STOP_BIT;
          end else begin
            r_RX_DV           <= 1'b1;
            r_DV_Extend_Count <= 3; // Hold o_RX_DV high for 3 clock cycles
            r_Clock_Count     <= 0;
            r_SM_Main         <= CLEANUP;
          end
        end

        CLEANUP: begin
          if (r_DV_Extend_Count > 0) begin
            r_DV_Extend_Count <= r_DV_Extend_Count - 1;
          end else begin
            r_RX_DV <= 1'b0;
            r_SM_Main <= IDLE;
          end
        end

        default: r_SM_Main <= IDLE;
      endcase
    end
  end

  assign o_RX_DV   = r_RX_DV;
  assign o_RX_Byte = r_RX_Byte;

endmodule


module uart_tx (
  input       i_Clock,
  input       i_Reset,       // Reset Signal
  input       i_Tx_DV,
  input [7:0] i_Tx_Byte, 
  input [13:0] i_CLKS_PER_BIT, // Input clock cycles per bit
  output      o_Tx_Active,
  output reg  o_Tx_Serial,
  output      o_Tx_Done
);

  // State Definitions
  parameter s_IDLE         = 3'b000;
  parameter s_TX_START_BIT = 3'b001;
  parameter s_TX_DATA_BITS = 3'b010;
  parameter s_TX_STOP_BIT  = 3'b011;
  parameter s_CLEANUP      = 3'b100;

  reg [2:0] r_SM_Main     = 0;
  reg [13:0] r_Clock_Count = 0;
  reg [2:0] r_Bit_Index   = 0;
  reg [7:0] r_Tx_Data     = 0;
  reg       r_Tx_Done     = 0;
  reg       r_Tx_Active   = 0;
  reg [13:0] r_Latched_CLKS_PER_BIT = 0; // Latched clock cycles per bit

  always @(posedge i_Clock or posedge i_Reset) begin
    if (i_Reset) begin
      r_SM_Main              <= s_IDLE;
      r_Clock_Count          <= 0;
      r_Bit_Index            <= 0;
      r_Tx_Data              <= 0;
      r_Tx_Done              <= 0;
      r_Tx_Active            <= 0;
      o_Tx_Serial            <= 1'b1; // Idle state for TX line
      //r_Latched_CLKS_PER_BIT <= i_CLKS_PER_BIT; // Latch CLKS_PER_BIT on reset
    end else begin
      r_Latched_CLKS_PER_BIT <= i_CLKS_PER_BIT; // Latch CLKS_PER_BIT on reset
      case (r_SM_Main)
        s_IDLE: begin
          o_Tx_Serial   <= 1'b1; // Line High for Idle
          r_Tx_Done     <= 1'b0;
          r_Clock_Count <= 0;
          r_Bit_Index   <= 0;

          if (i_Tx_DV == 1'b1) begin
            r_Tx_Active <= 1'b1;
            r_Tx_Data   <= i_Tx_Byte;
            r_SM_Main   <= s_TX_START_BIT;
          end else begin
            r_SM_Main <= s_IDLE;
          end
        end

        s_TX_START_BIT: begin
          o_Tx_Serial <= 1'b0; // Start Bit
          if (r_Clock_Count < r_Latched_CLKS_PER_BIT - 1) begin
            r_Clock_Count <= r_Clock_Count + 1;
            r_SM_Main     <= s_TX_START_BIT;
          end else begin
            r_Clock_Count <= 0;
            r_SM_Main     <= s_TX_DATA_BITS;
          end
        end

        s_TX_DATA_BITS: begin
          o_Tx_Serial <= r_Tx_Data[r_Bit_Index];
          if (r_Clock_Count < r_Latched_CLKS_PER_BIT - 1) begin
            r_Clock_Count <= r_Clock_Count + 1;
            r_SM_Main     <= s_TX_DATA_BITS;
          end else begin
            r_Clock_Count <= 0;
            if (r_Bit_Index < 7) begin
              r_Bit_Index <= r_Bit_Index + 1;
              r_SM_Main   <= s_TX_DATA_BITS;
            end else begin
              r_Bit_Index <= 0;
              r_SM_Main   <= s_TX_STOP_BIT;
            end
          end
        end

        s_TX_STOP_BIT: begin
          o_Tx_Serial <= 1'b1; // Stop Bit
          if (r_Clock_Count < r_Latched_CLKS_PER_BIT - 1) begin
            r_Clock_Count <= r_Clock_Count + 1;
            r_SM_Main     <= s_TX_STOP_BIT;
          end else begin
            r_Tx_Done     <= 1'b1;
            r_Clock_Count <= 0;
            r_Tx_Active   <= 1'b0;
            r_SM_Main     <= s_CLEANUP;
          end
        end

        s_CLEANUP: begin
          r_Tx_Done <= 1'b1;
          r_SM_Main <= s_IDLE;
        end

        default: r_SM_Main <= s_IDLE;
      endcase
    end
  end

  assign o_Tx_Active = r_Tx_Active;
  assign o_Tx_Done   = r_Tx_Done;

endmodule