`default_nettype none

module tt08_minimal_audio (
    input wire clk, rst_n, cpu_rst_n, tick,
    input wire acc_valid, acc_io, acc_write,
    input wire [19:0] acc_addr,
    input wire [31:0] acc_wdata,
    output wire acc_ready, acc_error,
    output wire [31:0] acc_rdata,
    output wire audio
);
    reg pending, completed, response_error;
    reg [1:0] command_addr;
    reg command_write;
    reg [31:0] command_data, response_data;
    reg [15:0] reload [0:2];
    reg [15:0] count [0:2];
    reg [3:0] level [0:2];
    reg [2:0] enabled;
    reg [2:0] phase [0:1];
    reg [14:0] lfsr;
    reg [3:0] prescale;
    reg [6:0] pwm_count, duty;
    reg muted;
    wire voice_tick = tick && prescale == 15;
    wire selected = acc_io && acc_addr >= 20'h30 && acc_addr <= 20'h33;
    wire valid_payload = acc_addr == 20'h33 ? ~|acc_wdata[31:2] :
                                                             ~|acc_wdata[31:22];
    wire request_error = !selected ||
        (acc_write ? !valid_payload : acc_addr != 20'h33);
    wire [31:0] status = {16'b0, 8'd1, 4'b0, enabled, muted};
    assign acc_ready = rst_n && cpu_rst_n && acc_valid && pending && !completed;
    assign acc_error = response_error;
    assign acc_rdata = response_data;
    wire commit = acc_ready && command_write && !response_error;

    always @(posedge clk) begin
        if (!rst_n || !cpu_rst_n) begin
            pending <= 0; completed <= 0; response_error <= 0;
            command_addr <= 0; command_write <= 0;
            command_data <= 0; response_data <= 0;
        end else if (!acc_valid) begin
            pending <= 0; completed <= 0;
        end else if (!pending && !completed) begin
            pending <= 1;
            command_addr <= acc_addr[1:0];
            command_write <= acc_write;
            command_data <= acc_wdata;
            response_error <= request_error;
            response_data <= !request_error && !acc_write ? status : 32'b0;
        end else if (acc_ready) begin
            pending <= 0; completed <= 1;
        end
    end

    function signed [7:0] signed_level;
        input active, positive;
        input [3:0] volume;
        reg signed [7:0] magnitude;
        begin
            magnitude = $signed({4'b0, volume});
            signed_level = !active ? 8'sd0 : positive ? magnitude : -magnitude;
        end
    endfunction
    wire signed [7:0] mix = signed_level(enabled[0], !phase[0][2], level[0]) +
                           signed_level(enabled[1], !phase[1][2], level[1]) +
                           signed_level(enabled[2], lfsr[14], level[2]);
    wire signed [7:0] next_duty = 8'sd64 + mix;
    assign audio = rst_n && !muted && pwm_count < duty;

    integer i;
    always @(posedge clk) begin
        if (!rst_n) begin
            prescale <= 0; pwm_count <= 0; duty <= 64; muted <= 1;
            enabled <= 0; lfsr <= 15'h0001;
            phase[0] <= 0; phase[1] <= 0;
            for (i=0; i<3; i=i+1) begin
                reload[i] <= 0; count[i] <= 0; level[i] <= 0;
            end
        end else begin
            if (tick) begin
                prescale <= prescale + 1'b1;
                pwm_count <= pwm_count + 1'b1;
                if (pwm_count == 127) duty <= next_duty[6:0];
            end
            if (commit && command_addr == 3) muted <= command_data[0];
            for (i=0; i<3; i=i+1) begin
                if (commit && command_addr == i) begin
                    reload[i] <= command_data[15:0] - 1'b1;
                    count[i] <= 0;
                    level[i] <= command_data[19:16];
                    enabled[i] <= command_data[20] && |command_data[15:0] &&
                                  |command_data[19:16];
                    if (command_data[21] || (!enabled[i] && command_data[20])) begin
                        if (i<2) phase[i] <= 0;
                        else lfsr <= 15'h0001;
                    end
                end else if (voice_tick && enabled[i]) begin
                    if (count[i] == reload[i]) begin
                        count[i] <= 0;
                        if (i<2) phase[i] <= phase[i] + 1'b1;
                        else lfsr <= {lfsr[13:0], lfsr[14] ^ lfsr[13]};
                    end else count[i] <= count[i] + 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire
