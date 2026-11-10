`default_nettype none

// APS6404L-3SQR SPI command / quad address+data, fixed 50 MHz host.
// One aligned little-endian word per transaction keeps CE low below 4 us.
module tta_psram_quad (
    input wire clk, rst_n,
    input wire valid, write,
    input wire [23:0] addr,
    input wire [31:0] wdata,
    output wire ready,
    output reg error,
    output reg [31:0] rdata,
    output reg cs_n, sck,
    input wire [3:0] dq_in,
    output reg [3:0] dq_out, dq_oe,
    output wire initializing
);
    localparam POWER=0, COMMAND=1, RESET_GAP=2, RESET_WAIT=3,
        IDLE=4, ADDRESS=5, DUMMY=6, DATA=7, FINISH=8, RESPONSE=9, GAP=10;
    reg [3:0] state;
    reg [12:0] delay_count;
    reg [3:0] count;
    reg [7:0] command;
    reg [23:0] address_shift;
    reg [31:0] data_shift, receive_shift;
    reg operation_write, reset_second, initialized;
    wire [31:0] wire_word = {wdata[7:0],wdata[15:8],wdata[23:16],wdata[31:24]};
    assign ready = rst_n && valid && state == RESPONSE;
    assign initializing = !initialized;
    task start_command;
        input [7:0] value;
        begin
            command <= value; count <= 0; cs_n <= 0; sck <= 0;
            dq_out <= {2'b11,1'b0,value[7]}; dq_oe <= 4'b1101;
            state <= COMMAND;
        end
    endtask
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= POWER; delay_count <= 0; count <= 0;
            command <= 0; address_shift <= 0;
            data_shift <= 0; receive_shift <= 0;
            operation_write <= 0; reset_second <= 0; initialized <= 0;
            cs_n <= 1; sck <= 0; dq_out <= 0; dq_oe <= 15;
            rdata <= 0; error <= 0;
        end else case (state)
            POWER: if (delay_count == 7999) begin
                delay_count <= 0; start_command(8'h66);
            end else delay_count <= delay_count + 1'b1;
            COMMAND: begin
                sck <= !sck;
                if (!sck) count <= count + 1'b1;
                else if (count == 8) begin
                    count <= 0;
                    if (!initialized) begin
                        cs_n <= 1; dq_oe <= 0; delay_count <= 0;
                        state <= reset_second ? RESET_WAIT : RESET_GAP;
                    end else begin
                        dq_oe <= 15; dq_out <= address_shift[23:20];
                        address_shift <= {address_shift[19:0],4'b0};
                        state <= ADDRESS;
                    end
                end else begin
                    command <= {command[6:0],1'b0};
                    dq_out <= {2'b11,1'b0,command[6]};
                end
            end
            RESET_GAP: if (delay_count == 2) begin
                reset_second <= 1; start_command(8'h99);
            end else delay_count <= delay_count + 1'b1;
            RESET_WAIT: if (delay_count == 2) begin
                initialized <= 1; state <= IDLE;
            end else delay_count <= delay_count + 1'b1;
            IDLE: if (valid) begin
                error <= |addr[1:0] || addr > 24'h7ffffc;
                if (|addr[1:0] || addr > 24'h7ffffc) state <= RESPONSE;
                else begin
                    operation_write <= write; address_shift <= addr;
                    data_shift <= wire_word; receive_shift <= 0;
                    start_command(write ? 8'h38 : 8'heb);
                end
            end
            ADDRESS: begin
                sck <= !sck;
                if (!sck) count <= count + 1'b1;
                else if (count == 6) begin
                    count <= 0;
                    if (operation_write) begin
                        state <= DATA; dq_out <= data_shift[31:28];
                        data_shift <= {data_shift[27:0],4'b0};
                    end else begin state <= DUMMY; dq_oe <= 0; end
                end else begin
                    dq_out <= address_shift[23:20];
                    address_shift <= {address_shift[19:0],4'b0};
                end
            end
            DUMMY: begin
                sck <= !sck;
                if (!sck) count <= count + 1'b1;
                else if (count == 6) begin count <= 0; state <= DATA; end
            end
            DATA: begin
                sck <= !sck;
                if (!sck) begin
                    count <= count + 1'b1;
                    if (!operation_write) receive_shift <= {receive_shift[27:0],dq_in};
                end else if (count == 8) begin
                    cs_n <= 1; dq_oe <= 0; delay_count <= 0; state <= FINISH;
                end else if (operation_write) begin
                    dq_out <= data_shift[31:28]; data_shift <= {data_shift[27:0],4'b0};
                end
            end
            FINISH: if (delay_count == 1) begin
                rdata <= {receive_shift[7:0],receive_shift[15:8],
                          receive_shift[23:16],receive_shift[31:24]};
                state <= RESPONSE;
            end else delay_count <= delay_count + 1'b1;
            RESPONSE: if (!valid) state <= GAP;
            GAP: if (!valid) state <= IDLE;
            default: begin cs_n <= 1; sck <= 0; dq_oe <= 0;
                error <= 1; state <= RESPONSE; end
        endcase
    end
endmodule
`default_nettype wire
