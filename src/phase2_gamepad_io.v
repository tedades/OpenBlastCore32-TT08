`default_nettype none

// DATA_IO target with an atomic snapshot and one completion per request.
module phase2_gamepad_io #(
    parameter integer WAIT_CYCLES = 0
) (
    input wire clk, rst_n,
    input wire pmod_data, pmod_clk, pmod_latch,
    input wire acc_valid, acc_io, acc_write,
    input wire [19:0] acc_addr,
    input wire [31:0] acc_wdata,
    output wire acc_ready, acc_error,
    output wire [31:0] acc_rdata
);
    generate if (WAIT_CYCLES < 0) begin : invalid_wait
        initial $fatal(1, "phase2_gamepad_io requires nonnegative WAIT_CYCLES");
    end endgenerate
    wire [1:0] b, y, select_button, start, up, down, left, right, a, x, l, r;
    wire [1:0] present;
    gamepad_pmod_dual decoder (
        .clk(clk), .rst_n(rst_n), .pmod_data(pmod_data),
        .pmod_clk(pmod_clk), .pmod_latch(pmod_latch),
        .b(b), .y(y), .select(select_button), .start(start), .up(up),
        .down(down), .left(left), .right(right), .a(a), .x(x),
        .l(l), .r(r), .is_present(present)
    );
    wire [11:0] slot0 = {b[0], y[0], select_button[0], start[0],
        up[0], down[0], left[0], right[0], a[0], x[0], l[0], r[0]};
    wire [11:0] slot1 = {b[1], y[1], select_button[1], start[1],
        up[1], down[1], left[1], right[1], a[1], x[1], l[1], r[1]};
    wire [31:0] live_snapshot = {6'b0, present, slot1, slot0};
    reg pending, completed, response_error;
    reg [31:0] response_data;
    localparam integer WAIT_WIDTH = WAIT_CYCLES > 0 ? $clog2(WAIT_CYCLES + 64'd1) : 1;
    wire [WAIT_WIDTH-1:0] remaining;
    generate if (WAIT_CYCLES == 0) begin : no_wait
        assign remaining = 0;
    end else begin : delayed
        reg [WAIT_WIDTH-1:0] count;
        assign remaining = count;
        always @(posedge clk) begin
            if (!rst_n) count <= 0;
            else if (acc_valid && !pending && !completed) count <= WAIT_CYCLES;
            else if (acc_valid && pending && !completed && count != 0)
                count <= count - 1'b1;
        end
    end endgenerate
    assign acc_ready = rst_n && acc_valid && pending && !completed && (remaining == 0);
    assign acc_error = response_error;
    assign acc_rdata = response_data;
    always @(posedge clk) begin
        if (!rst_n) begin
            pending <= 1'b0;
            completed <= 1'b0;
            response_error <= 1'b0;
            response_data <= 0;
        end else if (!acc_valid) begin
            pending <= 1'b0;
            completed <= 1'b0;
        end else if (!pending && !completed) begin
            pending <= 1'b1;
            response_error <= !acc_io || acc_write || (acc_addr != 20'h00020);
            response_data <= (acc_io && !acc_write && acc_addr == 20'h00020) ?
                             live_snapshot : 32'b0;
        end else if (acc_ready) begin
            pending <= 1'b0;
            completed <= 1'b1;
        end
    end
    wire unused_wdata = ^acc_wdata;
endmodule
`default_nettype wire
